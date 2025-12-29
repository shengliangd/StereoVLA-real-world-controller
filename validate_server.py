#!/usr/bin/env python3

"""
Simple Server Validation

This module validates that the model server is running and returns a valid response.

Author: Mi Yan
License: CC-BY-NC 4.0
Created: 2025-07-10
"""

import zmq
import numpy as np
import argparse


def validate_server(server_ip: str, port: int, timeout: int) -> bool:
    """
    Validate that the server is running and returns a valid dict.

    Args:
        server_ip: Server IP
        port: Server port
        timeout: Timeout in seconds

    Returns:
        True if server returns valid dict, False otherwise
    """
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout * 1000)

    try:
        socket.connect(f"tcp://{server_ip}:{port}")

        # Create test data matching agent.py format
        mock_image = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
        mock_proprio = [np.random.randn(7) for _ in range(4)]

        test_data = {
            'image_array': [mock_image],
            'image_wrist_array': [mock_image],
            'proprio_array': mock_proprio,
            'text': 'Validation test instruction',
        }

        socket.send_pyobj(test_data)
        print(f"tcp://{server_ip}:{port}")
        response = socket.recv_pyobj()

        # Check if response is a valid dict
        if not isinstance(response, dict):
            print(f"✗ Server returned {type(response)}, expected dict")
            return False

        print(f"✓ Server at {server_ip}:{port} returned valid dict")
        return True
    except zmq.Again:
        print(f"✗ Server at {server_ip}:{port} timeout after {timeout}s")
        return False
    except Exception as e:
        print(f"✗ Error connecting to server at {server_ip}:{port}: {e}")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validate that the model server is running and returns a valid response.")
    parser.add_argument("--ip", type=str, required=True, help="Server IP")
    parser.add_argument("--port", type=int, required=True, help="Server port")
    parser.add_argument("--timeout", type=int, default=5, help="Timeout in seconds (default: 5)")
    
    args = parser.parse_args()
    validate_server(args.ip, args.port, args.timeout)
