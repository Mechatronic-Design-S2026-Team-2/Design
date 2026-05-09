#!/usr/bin/env python3
"""Host-only HTTP control helper for selected Hexapod restart actions.

The ROS 2 operator node runs inside a host-networked Docker container and calls
this helper through http://127.0.0.1:18080. Keep this helper on the Jetson host,
not inside the container. It exposes only the limited restart actions used by
operator GUI buttons.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional


DEFAULT_ORBSLAM_SERVICE = 'orbslam2-realsense.service'
DEFAULT_MICROROS_SYSTEMD_SERVICES: List[str] = []
DEFAULT_MICROROS_CONTAINERS = ['microros_agent']
DEFAULT_DOCKER_COMPOSE_DIR = '/home/team2'
DEFAULT_DOCKER_COMPOSE_SERVICE = 'microros_agent'


def run_command(command: List[str], timeout: float = 12.0) -> Dict[str, Any]:
    started = time.time()
    try:
        proc = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
        return {
            'ok': proc.returncode == 0,
            'returncode': proc.returncode,
            'stdout': proc.stdout[-4000:],
            'stderr': proc.stderr[-4000:],
            'elapsed_sec': round(time.time() - started, 3),
            'command': command,
        }
    except Exception as exc:  # noqa: BLE001
        return {'ok': False, 'error': str(exc), 'command': command}


def service_exists(service: str) -> bool:
    result = run_command(['systemctl', 'show', service, '--property=LoadState', '--value'], timeout=3.0)
    return bool(result.get('ok')) and str(result.get('stdout', '')).strip() != 'not-found'


def restart_service(service: str) -> Dict[str, Any]:
    result = run_command(['systemctl', 'restart', service], timeout=25.0)
    result['service'] = service
    result['message'] = f"restart {'accepted' if result.get('ok') else 'failed'} for systemd service {service}"
    return result


def docker_container_exists(container: str) -> bool:
    if shutil.which('docker') is None:
      return False
    result = run_command(['docker', 'inspect', container], timeout=5.0)
    return bool(result.get('ok'))


def restart_docker_container(container: str) -> Dict[str, Any]:
    result = run_command(['docker', 'restart', container], timeout=30.0)
    result['container'] = container
    result['message'] = f"restart {'accepted' if result.get('ok') else 'failed'} for Docker container {container}"
    return result


def restart_docker_compose_service(compose_dir: str, service: str) -> Dict[str, Any]:
    compose_cmd: Optional[List[str]] = None
    if shutil.which('docker') is not None:
        compose_cmd = ['docker', 'compose']
    if compose_cmd is None:
        return {'ok': False, 'error': 'docker not found'}
    result = run_command(compose_cmd + ['restart', service], timeout=45.0)
    result['compose_dir'] = compose_dir
    result['compose_service'] = service
    result['message'] = f"restart {'accepted' if result.get('ok') else 'failed'} for compose service {service}"
    return result


class Handler(BaseHTTPRequestHandler):
    server_version = 'HexapodHostControl/0.2'
    orbslam_service = DEFAULT_ORBSLAM_SERVICE
    microros_systemd_services = list(DEFAULT_MICROROS_SYSTEMD_SERVICES)
    microros_containers = list(DEFAULT_MICROROS_CONTAINERS)
    docker_compose_dir = DEFAULT_DOCKER_COMPOSE_DIR
    docker_compose_service = DEFAULT_DOCKER_COMPOSE_SERVICE

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.client_address[0]} - {fmt % args}")

    def _send(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        encoded = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        self.send_response(int(status))
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def do_GET(self) -> None:  # noqa: N802
        if self.path.rstrip('/') == '/status':
            containers = {}
            for container in self.microros_containers:
                containers[container] = docker_container_exists(container)
            self._send({
                'ok': True,
                'status': 'ready',
                'orbslam_service': self.orbslam_service,
                'microros_systemd_services': self.microros_systemd_services,
                'microros_containers': self.microros_containers,
                'microros_container_exists': containers,
                'docker_compose_dir': self.docker_compose_dir,
                'docker_compose_service': self.docker_compose_service,
            })
            return
        self._send({'ok': False, 'error': 'not found'}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.rstrip('/')
        if path == '/restart/orbslam2':
            self._send(restart_service(self.orbslam_service))
            return
        if path == '/restart/microros_agent':
            tried: List[Dict[str, Any]] = []

            # Prefer the compose/container deployment supplied for this robot.
            for container in self.microros_containers:
                if docker_container_exists(container):
                    self._send(restart_docker_container(container))
                    return
                tried.append({'container': container, 'exists': False})

            # Fall back to docker compose restart if the container is not up yet.
            if self.docker_compose_service:
                result = restart_docker_compose_service(self.docker_compose_dir, self.docker_compose_service)
                if result.get('ok'):
                    self._send(result)
                    return
                tried.append(result)

            # Last fallback: older installations with a host-side systemd service.
            for service in self.microros_systemd_services:
                if service_exists(service):
                    self._send(restart_service(service))
                    return
                tried.append({'service': service, 'exists': False})

            self._send({
                'ok': False,
                'error': 'no restartable micro-ROS Agent target found',
                'tried': tried,
            }, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        self._send({'ok': False, 'error': 'not found'}, HTTPStatus.NOT_FOUND)


def split_csv(value: str) -> List[str]:
    return [part.strip() for part in value.split(',') if part.strip()]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18080)
    parser.add_argument('--orbslam-service', default=DEFAULT_ORBSLAM_SERVICE)
    parser.add_argument('--microros-systemd-services', default=','.join(DEFAULT_MICROROS_SYSTEMD_SERVICES))
    parser.add_argument('--microros-containers', default=','.join(DEFAULT_MICROROS_CONTAINERS))
    parser.add_argument('--docker-compose-dir', default=DEFAULT_DOCKER_COMPOSE_DIR)
    parser.add_argument('--docker-compose-service', default=DEFAULT_DOCKER_COMPOSE_SERVICE)
    args = parser.parse_args()
    Handler.orbslam_service = args.orbslam_service
    Handler.microros_systemd_services = split_csv(args.microros_systemd_services)
    Handler.microros_containers = split_csv(args.microros_containers)
    Handler.docker_compose_dir = args.docker_compose_dir
    Handler.docker_compose_service = args.docker_compose_service
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"hexapod host control listening on http://{args.bind}:{args.port}")
    server.serve_forever()


if __name__ == '__main__':
    main()
