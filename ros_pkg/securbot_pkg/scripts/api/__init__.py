"""Provide modules to interface with SecurBot database."""

import os

server_url = os.environ["SECURBOT_SERVER_URL"]
api_path = os.environ["SECURBOT_API_PATH"]

api_url = server_url + api_path
