import feedparser
import datetime
import subprocess
import requests
from bs4 import BeautifulSoup
import json
import re
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from html import unescape
from html.parser import HTMLParser
import email.utils
from pull_feed import FetchRSS, FetchGithubBlogs
from pull_google_news import FetchGoogleNews
from filters import filter_min_content_length, generate_consolidated_summary
from dataclasses import dataclass
from typing import List
from functools import partial
import slack_sender

TIME_RANGE = 1
DEFAULT_MODEL = "qwen2.5:7b"
ACADEMIC_MODEL = "glm4"


def summarize_item(item: dict) -> str:
    """Run an Ollama model to summarize or explain an item.
    Academic posts (from FetchGithubBlogs) get a concept-explanation prompt
    with GLM; all others get a news-summary prompt with the default model.
    """
    if item.get("is_academic"):
        model = ACADEMIC_MODEL
        prompt = (
            f"You are reading a technical/academic blog post. "
            f"Explain the key concepts in 3-5 sentences. "
            f"Include important mathematical ideas or equations where helpful.\n\n"
            f"Title: {item['title']}\n"
            f"Content: {item['content']}\n\n"
            f"Output ONLY the explanation, nothing else."
        )
    else:
        model = DEFAULT_MODEL
        prompt = (
            f"Summarize this article in 2-3 sentences. "
            f"Include the key details: who, what, when, where, why. Be specific and informative.\n\n"
            f"Title: {item['title']}\n"
            f"Content: {item['content']}\n\n"
            f"Output ONLY the summary, nothing else."
        )
    result = subprocess.run(
        ["ollama", "run", model],
        input=prompt,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


fetchers = [
    # FetchRSS(name="UBC", url="https://www.reddit.com/r/UBC/.rss", days=TIME_RANGE),
    # FetchRSS(
    #     name="Jeff Geerling",
    #     url="https://www.jeffgeerling.com/blog.xml",
    #     days=TIME_RANGE,
    # ),
    # FetchGoogleNews(
    #     name="Nauticus Robotics (Google News)",
    #     query="Nauticus Robotics",
    #     days=TIME_RANGE,
    #     additional_per_item_filters=[partial(filter_min_content_length, min_chars=200)],
    #     additional_total_filters=[generate_consolidated_summary],
    # ),
]

academic_fetchers = [
    FetchGithubBlogs(name="aipiano", base_url="https://aipiano.github.io"),
    FetchGithubBlogs(name="udohsolomon", base_url="https://udohsolomon.github.io"),
    FetchGithubBlogs(name="adaning", base_url="https://adaning.github.io"),
]

if __name__ == "__main__":

    all_items = []

    for fetcher in fetchers:
        all_items.extend(fetcher.fetch())

    # Academic fetchers: drain one blog at a time (oldest→newest).
    # Try each in order; stop as soon as one yields a post.
    for fetcher in academic_fetchers:
        academic_items = fetcher.fetch()
        if academic_items:
            all_items.extend(academic_items)
            break

    # Group by source
    sources = {}
    for item in all_items:
        source = item["source"]
        if source not in sources:
            sources[source] = []
        sources[source].append(item)

    # Build Slack message
    slack_message_parts = []

    for source, items in sources.items():
        print(f"\n{source}:")
        source_summaries = []

        # Check if there's a consolidated summary
        consolidated_summary = None
        if items and "consolidated_summary" in items[0]:
            consolidated_summary = items[0]["consolidated_summary"]
            print(f"\n{'='*80}")
            print(f"CONSOLIDATED SUMMARY ({len(items)} related articles):")
            print(f"{'='*80}")
            print(consolidated_summary)
            print(f"{'='*80}\n")

        for item in items:
            print(f"\n{'='*80}")
            print(f"{item['title']} ({item['link']})")
            print(f"{'='*80}")

            summary = summarize_item(item)

            print(f"\nSummary: {summary}")

            # Use Slack link format: <url|text>
            source_summaries.append(f"• <{item['link']}|{item['title']}>: {summary}\n")

        print("\n" + "=" * 60)

        # Add to Slack message
        slack_message_parts.append(f"*{source}*\n")
        if consolidated_summary:
            slack_message_parts.append(f"_{consolidated_summary}_\n\n")
        slack_message_parts.append("\n".join(source_summaries))
        slack_message_parts.append("\n\n")

    # Send to Slack
    slack_message = "".join(slack_message_parts)
    slack_sender.send(slack_message)
