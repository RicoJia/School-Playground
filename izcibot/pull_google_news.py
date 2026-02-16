import json
import re
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone
import email.utils
from html import unescape
from html.parser import HTMLParser
from bs4 import BeautifulSoup
import subprocess
from functools import partial
from pull_feed import FetcherBase


GOOGLE_NEWS_RSS_BASE = "https://news.google.com/rss/search"
GOOGLE_NEWS_DECODE_ENDPOINT = (
    "https://news.google.com/_/DotsSplashUi/data/batchexecute?rpcids=Fbv4je"
)
USER_AGENT = (
    "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"
)


def clean_text(value: str) -> str:
    value = unescape(value)
    value = re.sub(r"\s+", " ", value).strip()
    return value


class ParagraphExtractor(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.in_ignored_tag = False
        self.in_paragraph = False
        self.paragraph_buffer: list[str] = []
        self.paragraphs: list[str] = []

    def handle_starttag(self, tag: str, attrs) -> None:
        if tag in {"script", "style", "noscript"}:
            self.in_ignored_tag = True
            return
        if tag == "p":
            self.in_paragraph = True
            self.paragraph_buffer = []

    def handle_endtag(self, tag: str) -> None:
        if tag in {"script", "style", "noscript"}:
            self.in_ignored_tag = False
            return
        if tag == "p" and self.in_paragraph:
            paragraph = clean_text("".join(self.paragraph_buffer))
            if paragraph:
                self.paragraphs.append(paragraph)
            self.in_paragraph = False
            self.paragraph_buffer = []

    def handle_data(self, data: str) -> None:
        if self.in_ignored_tag:
            return
        if self.in_paragraph:
            self.paragraph_buffer.append(data)


class FetchGoogleNews(FetcherBase):
    def __init__(
        self,
        name,
        query="Nauticus Robotics",
        days=30,
        limit=25,
        content_chars=1000,
        no_content=False,
        additional_per_item_filters=None,
        additional_total_filters=None,
    ):
        super().__init__(name, days)
        self.query = query
        self.limit = limit
        self.content_chars = content_chars
        self.no_content = no_content
        self.additional_per_item_filters = additional_per_item_filters or []
        self.additional_total_filters = additional_total_filters or []

    def build_rss_url(self, query: str) -> str:
        params = {
            "q": query,
            "hl": "en-US",
            "gl": "US",
            "ceid": "US:en",
        }
        return f"{GOOGLE_NEWS_RSS_BASE}?{urllib.parse.urlencode(params)}"

    def parse_pub_date(self, raw: str | None) -> datetime | None:
        if not raw:
            return None
        try:
            dt = email.utils.parsedate_to_datetime(raw)
            if dt is None:
                return None
            if dt.tzinfo is None:
                return dt.replace(tzinfo=timezone.utc)
            return dt.astimezone(timezone.utc)
        except (TypeError, ValueError):
            return None

    def fetch_url(self, url: str, timeout: int = 15) -> tuple[bytes, str, str]:
        request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = response.read()
            final_url = response.geturl()
            content_type = response.headers.get("Content-Type", "")
        return body, final_url, content_type

    def decode_google_news_link(self, google_link: str) -> str:
        parsed = urllib.parse.urlparse(google_link)
        if parsed.netloc != "news.google.com" or "/rss/articles/" not in parsed.path:
            return google_link

        article_id = parsed.path.split("/rss/articles/", maxsplit=1)[1]
        if not article_id:
            return google_link

        try:
            article_html, _, _ = self.fetch_url(google_link)
            article_html_text = article_html.decode("utf-8", errors="replace")
            ts_match = re.search(r'data-n-a-ts="(\d+)"', article_html_text)
            sg_match = re.search(r'data-n-a-sg="([^"]+)"', article_html_text)
            if not ts_match or not sg_match:
                return google_link

            ts = int(ts_match.group(1))
            signature = sg_match.group(1)
            request_payload = [
                "garturlreq",
                [
                    [
                        "en-US",
                        "US",
                        ["FINANCE_TOP_INDICES", "WEB_TEST_1_0_0"],
                        None,
                        None,
                        1,
                        1,
                        "US:en",
                        None,
                        180,
                        None,
                        None,
                        None,
                        None,
                        None,
                        0,
                        None,
                        None,
                        [1608992183, 723341000],
                    ],
                    "en-US",
                    "US",
                    1,
                    [2, 3, 4, 8],
                    1,
                    0,
                    "655000234",
                    0,
                    0,
                    None,
                    0,
                ],
                article_id,
                ts,
                signature,
            ]

            rpc_payload = [
                [
                    [
                        "Fbv4je",
                        json.dumps(request_payload, separators=(",", ":")),
                        None,
                        "generic",
                    ]
                ]
            ]
            body = urllib.parse.urlencode(
                {"f.req": json.dumps(rpc_payload, separators=(",", ":"))}
            ).encode()
            request = urllib.request.Request(
                GOOGLE_NEWS_DECODE_ENDPOINT,
                data=body,
                headers={
                    "Content-Type": "application/x-www-form-urlencoded;charset=UTF-8",
                    "User-Agent": USER_AGENT,
                },
            )
            with urllib.request.urlopen(request, timeout=15) as response:
                response_text = response.read().decode("utf-8", errors="replace")

            candidate_urls = re.findall(r"https?://[^\"\\\]\s]+", response_text)
            for decoded in candidate_urls:
                if "news.google.com/" not in decoded:
                    return decoded
            return google_link
        except Exception:
            return google_link

    def extract_article_content(
        self, body: bytes, content_type: str, max_chars: int
    ) -> str:
        charset = "utf-8"
        match = re.search(r"charset=([\w\-]+)", content_type, flags=re.IGNORECASE)
        if match:
            charset = match.group(1).strip()
        try:
            html = body.decode(charset, errors="replace")
        except LookupError:
            html = body.decode("utf-8", errors="replace")

        parser = ParagraphExtractor()
        parser.feed(html)

        text = " ".join(p for p in parser.paragraphs if len(p) > 40)
        if not text:
            # Fallback: use BeautifulSoup to extract all text
            soup = BeautifulSoup(html, "html.parser")
            # Remove script and style
            for tag in soup(["script", "style", "noscript"]):
                tag.decompose()
            text = soup.get_text(separator=" ", strip=True)
            text = re.sub(r"\s+", " ", text).strip()

        if not text:
            return "Unable to extract readable article text."
        if len(text) <= max_chars:
            return text
        return f"{text[:max_chars].rstrip()}..."

    def fetch(self):
        cutoff = datetime.now(timezone.utc) - timedelta(days=self.days)
        rss_url = self.build_rss_url(self.query)

        try:
            with urllib.request.urlopen(rss_url, timeout=15) as response:
                content = response.read()
        except Exception as exc:
            print(f"Failed to fetch feed: {exc}")
            return []

        try:
            root = ET.fromstring(content)
        except ET.ParseError as exc:
            print(f"Failed to parse XML: {exc}")
            return []

        items = []
        for entry in root.findall("./channel/item"):
            title = (entry.findtext("title") or "Untitled").strip()
            link = (entry.findtext("link") or "").strip()
            pub_dt = self.parse_pub_date((entry.findtext("pubDate") or "").strip())
            source = entry.find("source")
            source_name = (
                source.text.strip()
                if source is not None and source.text
                else "Unknown source"
            )

            if pub_dt and pub_dt < cutoff:
                continue

            items.append(
                {
                    "title": title,
                    "source": self.name,
                    "link": link,
                    "pub_dt": pub_dt,
                    "resolved_link": "",
                    "content": "",
                }
            )

        items.sort(
            key=lambda x: x["pub_dt"] or datetime.min.replace(tzinfo=timezone.utc),
            reverse=True,
        )
        items = items[: self.limit]

        if not self.no_content:
            for item in items:
                try:
                    resolved_link = self.decode_google_news_link(item["link"])
                    body, resolved_link, content_type = self.fetch_url(resolved_link)
                    item["resolved_link"] = resolved_link
                    item["content"] = self.extract_article_content(
                        body, content_type, self.content_chars
                    )
                except urllib.error.URLError as exc:
                    item["resolved_link"] = "Unavailable"
                    item["content"] = f"Failed to fetch article ({exc})."
                except Exception as exc:
                    item["resolved_link"] = "Unavailable"
                    item["content"] = (
                        f"Unexpected error while extracting content ({exc})."
                    )

        for item_filter in self.additional_per_item_filters:
            items = [item for item in items if item_filter(item)]

        for total_filter in self.additional_total_filters:
            items = total_filter(items)

        return items


if __name__ == "__main__":
    from filters import filter_min_content_length, generate_consolidated_summary

    fetcher = FetchGoogleNews(
        name="Nauticus Robotics (Google News)",
        query="Nauticus Robotics",
        days=10,
        additional_per_item_filters=[partial(filter_min_content_length, min_chars=200)],
        additional_total_filters=[generate_consolidated_summary],
    )
    news = fetcher.fetch()
    print(f"\nGoogle News results for {fetcher.query}: {len(news)} article(s)\n")
    for idx, item in enumerate(news, start=1):
        pub_date = (
            item["pub_dt"].strftime("%Y-%m-%d %H:%M UTC")
            if item["pub_dt"]
            else "Unknown date"
        )
        print(f"{idx}. {item['title']}")
        print(f"   Source: {item['source']}")
        print(f"   Published: {pub_date}")
        print(f"   Google News Link: {item['link']}")
        if not fetcher.no_content:
            print(f"   Article Link: {item['resolved_link']}")
            print(f"   Content: {item['content']}\n")
