# Copyright 2025 ujwalwag
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""JointState → yaml text."""

# use gz or jsp gui
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# joint rename
DEFAULT_JOINT_MAP = {
    'shoulder_joint': 'shoulder',
    'elbow_joint': 'elbow',
    'wrist_joint': 'wrist',
}

_DEFAULT_TOPICS = ('/joint_states', '/joint_states_gz')


def _make_qos(best_effort: bool) -> QoSProfile:
    rel = ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE
    return QoSProfile(depth=20, reliability=rel)


class _JointStatesSnapshot(Node):
    def __init__(
        self, joint_map: Dict[str, str], topics: List[str], best_effort_qos: bool
    ) -> None:
        super().__init__('joint_states_snapshot')
        self._joint_map = joint_map
        self._latest: Optional[JointState] = None
        self._latest_topic: str = ''
        qos = _make_qos(best_effort_qos)
        for t in topics:
            self.create_subscription(
                JointState, t, self._make_cb(t), qos
            )
            self.get_logger().info(
                f'Listening on {t} (QoS {"BEST_EFFORT" if best_effort_qos else "RELIABLE"})'
            )

    def _make_cb(self, topic: str):
        def _cb(msg: JointState) -> None:
            names = set(msg.name)
            if not self._joint_map.keys() <= names:
                return
            self._latest = msg
            self._latest_topic = topic

        return _cb

    def triplet_from_latest(self) -> Optional[Dict[str, float]]:
        if self._latest is None:
            return None
        msg = self._latest
        pos: Dict[str, float] = {}
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                break
            if name in self._joint_map:
                pos[self._joint_map[name]] = float(msg.position[i])
        if len(pos) != len(self._joint_map):
            return None
        return {k: pos[k] for k in ('shoulder', 'elbow', 'wrist')}


def _round_triplet(t: Dict[str, float], decimals: int = 5) -> Dict[str, float]:
    out: Dict[str, float] = {}
    eps = 10.0 ** (-max(2, decimals))
    for k, v in t.items():
        r = round(float(v), decimals)
        if abs(r) < eps:
            r = 0.0
        out[k] = r
    return out


def _format_output(
    triplet: Dict[str, float], fmt: str, stage_label: Optional[str]
) -> str:
    triplet = _round_triplet(triplet)
    sh, el, wr = triplet['shoulder'], triplet['elbow'], triplet['wrist']
    if fmt == 'json':
        blob: Dict[str, object] = {stage_label: triplet} if stage_label else triplet
        return json.dumps(blob, indent=2) + '\n'
    lines: List[str] = []
    if stage_label:
        lines.append(f'# arm.stages {stage_label}')
        lines.append(f'{stage_label}:')
        lines.append(f'  shoulder: {sh:.5f}')
        lines.append(f'  elbow: {el:.5f}')
        lines.append(f'  wrist: {wr:.5f}')
    else:
        lines.append('# radians')
        lines.append(f'shoulder: {sh:.5f}')
        lines.append(f'elbow: {el:.5f}')
        lines.append(f'wrist: {wr:.5f}')
    return '\n'.join(lines) + '\n'


def _help_no_publishers() -> str:
    return (
        'No arm JointState. Try --topic /joint_states_gz or jsp_gui + rsp.\n'
        'ros2 topic echo /joint_states --once\n'
        '--settle 1.0 if lag.'
    )


def main() -> None:
    parser = argparse.ArgumentParser(description='JointState dump.')
    parser.add_argument(
        '--format',
        choices=('yaml', 'json'),
        default='yaml',
        help='fmt',
    )
    parser.add_argument('--output', '-o', default=None, help='out file')
    parser.add_argument(
        '--stage',
        default=None,
        metavar='NAME',
        help='stage key',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=15.0,
        help='wait s',
    )
    parser.add_argument(
        '--settle',
        type=float,
        default=0.75,
        help='listen s',
    )
    parser.add_argument(
        '--topic',
        action='append',
        default=None,
        metavar='NAME',
        help='topic repeat',
    )
    parser.add_argument(
        '--qos-reliable',
        action='store_true',
        help='reliable qos',
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='log rad',
    )
    args = parser.parse_args()

    topics = args.topic if args.topic else list(_DEFAULT_TOPICS)

    rclpy.init()
    node = _JointStatesSnapshot(DEFAULT_JOINT_MAP, topics, not args.qos_reliable)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.triplet_from_latest() is not None:
                break
        else:
            node.get_logger().error('Timeout waiting for a complete JointState.')
            sys.stderr.write(_help_no_publishers() + '\n')
            sys.exit(1)

        settle_end = time.monotonic() + max(0.0, args.settle)
        while rclpy.ok() and time.monotonic() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)

        triplet = node.triplet_from_latest()
        if triplet is None:
            node.get_logger().error('Lost JointState before export.')
            sys.exit(1)

        if args.verbose:
            node.get_logger().info(
                f'Last JointState from {node._latest_topic!r}: '
                f'shoulder={triplet["shoulder"]:.6f} elbow={triplet["elbow"]:.6f} '
                f'wrist={triplet["wrist"]:.6f}'
            )

        text = _format_output(triplet, args.format, args.stage)
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write(text)
            node.get_logger().info(f'Wrote {args.output}')
        else:
            print(text, end='')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
