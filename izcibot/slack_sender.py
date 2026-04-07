#!/usr/bin/env python3
from slack_sdk import WebClient
import os
from dotenv import load_dotenv

load_dotenv()

# Slack configuration - loaded from .env
SLACK_BOT_TOKEN = os.getenv("SLACK_BOT_TOKEN")
SLACK_MEMBER_ID = os.getenv("SLACK_MEMBER_ID")


def send(message):
    """Send a message to Slack.

    Args:
        message: The text message to send (supports Slack formatting)

    Returns:
        True if successful, False otherwise
    """
    if not message or not message.strip():
        print("No content to send to Slack.")
        return False

    client = WebClient(token=SLACK_BOT_TOKEN)
    try:
        client.chat_postMessage(channel=SLACK_MEMBER_ID, text=message)
        print("\n✓ Message sent to Slack successfully!")
        return True
    except Exception as e:
        print(f"\n✗ Failed to send message to Slack: {e}")
        return False


if __name__ == "__main__":
    import time

    test_message = f"<https://example.com|Test message> from RSS bot {time.time()}"
    send(test_message)
