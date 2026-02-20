import feedparser
import datetime
import re
import sqlite3
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, unquote


TIME_RANGE = 10


class FetcherBase:
    def __init__(self, name, days=TIME_RANGE):
        self.name = name
        self.days = days

    def fetch(self):
        """Abstract method to fetch items. Should return list of dicts with keys: title, link, pub_date, content, source"""
        raise NotImplementedError


def fetch_article_content(url):
    """Fetch and extract main text content from a webpage."""
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, "html.parser")

        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()

        # Try to find main content (common patterns)
        article = (
            soup.find("article") or soup.find("main") or soup.find(class_="content")
        )
        if article:
            text = article.get_text(separator=" ", strip=True)
        else:
            text = soup.get_text(separator=" ", strip=True)

        # Clean up whitespace
        lines = (line.strip() for line in text.splitlines())
        text = " ".join(line for line in lines if line)

        # Limit to first 3000 words to avoid overwhelming the model
        words = text.split()[:3000]
        return " ".join(words)
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return None


class FetchRSS(FetcherBase):
    def __init__(self, name, url, days=TIME_RANGE, fetch_content=True):
        super().__init__(name, days)
        self.url = url
        self.fetch_content = fetch_content

    def fetch(self):
        current_date = datetime.datetime.now()
        cutoff = current_date - datetime.timedelta(days=self.days)

        all_items = []

        feed = feedparser.parse(self.url)
        recent_items = []
        for entry in feed.entries:
            pub_date = None
            if hasattr(entry, "published_parsed") and entry.published_parsed:
                pub_date = datetime.datetime(*entry.published_parsed[:6])
            if hasattr(entry, "updated_parsed") and entry.updated_parsed:
                updated_date = datetime.datetime(*entry.updated_parsed[:6])
                if pub_date is None or updated_date > pub_date:
                    pub_date = updated_date
            if pub_date and pub_date > cutoff:
                content = ""
                if self.fetch_content:
                    content = fetch_article_content(entry.link) or getattr(
                        entry, "description", ""
                    )
                recent_items.append(
                    {
                        "title": entry.title,
                        "link": entry.link,
                        "pub_date": pub_date,
                        "content": content,
                        "source": self.name,
                    }
                )

        all_items.extend(recent_items)

        # Sort by date
        all_items.sort(key=lambda x: x["pub_date"], reverse=True)
        return all_items


class FetchGithubBlogs(FetcherBase):
    """
    Scrape one or more GitHub Pages / Jekyll blogs.

    For each base URL:
      1. Crawl the index (and /page/N/ pagination) to collect post URLs.
      2. Identify post URLs by the Jekyll date path /YYYY/MM/DD/.
      3. Sort posts newest-first by that date.
      4. Return posts as dicts compatible with the rest of the pipeline.
    """

    _DATE_RE = re.compile(r"/(\d{4})/(\d{2})/(\d{2})/")

    def __init__(self, name: str, base_urls: list[str], days: int = 100000):
        super().__init__(name, days=days)
        self.base_urls = base_urls

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _collect_post_urls(self, base_url: str) -> list[tuple[datetime.date, str]]:
        """Return (post_date, abs_url) pairs from the blog index, newest-first."""
        session = requests.Session()
        session.headers.update({"User-Agent": "Mozilla/5.0"})

        seen: set[str] = set()
        page = 1

        while True:
            url = base_url if page == 1 else f"{base_url.rstrip('/')}/page/{page}/"
            r = session.get(url, timeout=20)
            if r.status_code == 404:
                break
            r.raise_for_status()
            soup = BeautifulSoup(r.text, "html.parser")

            found_new = False
            for a in soup.find_all("a", href=True):
                abs_url = urljoin(base_url, a["href"]).rstrip("/")
                m = self._DATE_RE.search(abs_url)
                if m and abs_url not in seen:
                    seen.add(abs_url)
                    found_new = True

            if not found_new:
                break
            page += 1

        items: list[tuple[datetime.date, str]] = []
        for u in seen:
            m = self._DATE_RE.search(u)
            if m:
                post_date = datetime.date(
                    int(m.group(1)), int(m.group(2)), int(m.group(3))
                )
                items.append((post_date, u))

        items.sort(key=lambda x: x[0], reverse=True)  # newest first
        return items

    def _scrape_post(self, url: str) -> tuple[str, str]:
        """Fetch a post page and return (title, body_text)."""
        try:
            r = requests.get(url, timeout=10, headers={"User-Agent": "Mozilla/5.0"})
            r.raise_for_status()
            soup = BeautifulSoup(r.content, "html.parser")
            for tag in soup(["script", "style"]):
                tag.decompose()
            h1 = soup.find("h1")
            title = (
                h1.get_text(strip=True)
                if h1
                else unquote(url.rstrip("/").split("/")[-1]).replace("-", " ")
            )
            content_el = (
                soup.find("article")
                or soup.find("main")
                or soup.find(class_="post-content")
                or soup.find(class_="content")
                or soup.find("body")
            )
            body = content_el.get_text(separator=" ", strip=True) if content_el else ""
            words = body.split()[:3000]
            return title, " ".join(words)
        except Exception as e:
            print(f"Error fetching {url}: {e}")
            slug = unquote(url.rstrip("/").split("/")[-1]).replace("-", " ")
            return slug, ""

    # ------------------------------------------------------------------
    # FetcherBase interface
    # ------------------------------------------------------------------

    def fetch(self) -> list[dict]:
        cutoff = datetime.datetime.now() - datetime.timedelta(days=self.days)
        items: list[dict] = []

        for base_url in self.base_urls:
            dated_posts = self._collect_post_urls(base_url)

            for post_date, post_url in dated_posts:
                pub_dt = datetime.datetime.combine(post_date, datetime.time.min)
                if pub_dt < cutoff:
                    continue

                title, content = self._scrape_post(post_url)
                items.append(
                    {
                        "title": title,
                        "link": post_url,
                        "pub_date": pub_dt,
                        "content": content,
                        "source": self.name,
                    }
                )

        items.sort(key=lambda x: x["pub_date"])
        return items


class DatabaseManager:
    def __init__(self, db_path: str):
        self.db_path = db_path
        self._ensure_schema()

    def _get_db_conn(self) -> sqlite3.Connection:
        return sqlite3.connect(self.db_path)

    def _ensure_schema(self):
        conn = self._get_db_conn()
        try:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS seen_github_posts (
                    url        TEXT PRIMARY KEY,
                    first_seen TEXT NOT NULL
                )
                """
            )
            conn.commit()
        finally:
            conn.close()

    def entry_exists(self, url: str) -> bool:
        conn = self._get_db_conn()
        try:
            row = conn.execute(
                "SELECT 1 FROM seen_github_posts WHERE url = ?",
                (url,),
            ).fetchone()
            return row is not None
        finally:
            conn.close()

    def mark_as_seen(self, url: str):
        conn = self._get_db_conn()
        try:
            conn.execute(
                "INSERT OR IGNORE INTO seen_github_posts (url, first_seen) VALUES (?, ?)",
                (url, datetime.datetime.now(datetime.UTC).isoformat()),
            )
            conn.commit()
        finally:
            conn.close()


if __name__ == "__main__":
    # fetcher = FetchRSS(name="UBC", url="https://www.reddit.com/r/UBC/.rss")
    # items = fetcher.fetch()
    # print(f"Fetched {len(items)} items from {fetcher.name}")
    # for item in items:
    #     print(f"{item['title']} ({item['link']})")

    fetcher = FetchGithubBlogs(
        name="Github Blogs",
        base_urls=[
            "https://aipiano.github.io",
            # "https://udohsolomon.github.io/"
            # "https://adaning.github.io/"
        ],
    )
    items = fetcher.fetch()
    print(f"Fetched {len(items)} items from {fetcher.name}")
    db_mgr = DatabaseManager("test_github_blogs.db")
    for item in items:
        if not db_mgr.entry_exists(item["link"]):
            print(f"New post: {item['title']}")
            db_mgr.mark_as_seen(item["link"])
            break
