#!/usr/bin/env python3
"""RUCHE entry point."""

import argparse
import logging
import sys

from app import create_app

logging.basicConfig(level=logging.INFO)
LOG = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description="RUCHE - Robot Control via LLM")
    parser.add_argument("--port", type=int, default=7860, help="Gradio server port")
    parser.add_argument("--dev", action="store_true", help="Development mode")
    args = parser.parse_args()

    app = create_app()
    app.launch(server_port=args.port, debug=args.dev)


if __name__ == "__main__":
    main()
