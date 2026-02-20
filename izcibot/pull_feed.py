import feedparser
import datetime
import re
import sqlite3
import xml.etree.ElementTree as ET
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
    Scrape a single GitHub Pages / Jekyll blog.

    1. Try /sitemap.xml — parse <loc> + <lastmod> for accurate post dates.
    2. Fall back to crawling the index pages if no sitemap is available.
       Handles both /page/2/ and /page2/ pagination styles.
       Detects posts by Jekyll date path (/YYYY/MM/DD/) or by heading links.
    3. Sort posts newest-first by date.
    4. Return posts as dicts compatible with the rest of the pipeline.
    """

    _DATE_RE = re.compile(r"/(\d{4})/(\d{2})/(\d{2})/")
    _SITEMAP_NS = "http://www.sitemaps.org/schemas/sitemap/0.9"
    # URL path segments that indicate non-post pages
    _SKIP_SEGMENTS = (
        "/tags",
        "/categories",
        "/about",
        "/feed",
        "/page",
        "/search",
        "/archive",
        "/_posts",
    )

    def __init__(self, name: str, base_url: str, days: int = 100000):
        super().__init__(name, days=days)
        self.base_url = base_url

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _session(self) -> requests.Session:
        s = requests.Session()
        s.headers.update({"User-Agent": "Mozilla/5.0"})
        return s

    def _try_sitemap(self, base_url: str) -> list[tuple[datetime.date, str]] | None:
        """Fetch and parse /sitemap.xml. Returns None if unavailable."""
        sitemap_url = base_url.rstrip("/") + "/sitemap.xml"
        try:
            r = self._session().get(sitemap_url, timeout=10)
            if r.status_code != 200:
                return None
            root = ET.fromstring(r.content)
            ns = {"sm": self._SITEMAP_NS}
            items: list[tuple[datetime.date, str]] = []
            for url_el in root.findall("sm:url", ns):
                loc_el = url_el.find("sm:loc", ns)
                lastmod_el = url_el.find("sm:lastmod", ns)
                if loc_el is None or not loc_el.text:
                    continue
                loc = loc_el.text.strip().rstrip("/")
                # Skip obvious non-post URLs
                path = loc.replace(base_url.rstrip("/"), "")
                if any(path.startswith(s) for s in self._SKIP_SEGMENTS):
                    continue
                if loc == base_url.rstrip("/"):
                    continue
                # Skip single-segment paths like /sitemap, /about, /portfolio
                if path.count("/") < 2:
                    continue
                # Skip index/listing pages (e.g. Hexo's /markdown/index.html)
                if loc.split("/")[-1] == "index.html":
                    continue
                # Parse date from lastmod, then URL, then today as fallback
                post_date = None
                if lastmod_el is not None and lastmod_el.text:
                    try:
                        post_date = datetime.date.fromisoformat(
                            lastmod_el.text.strip()[:10]
                        )
                    except ValueError:
                        pass
                if post_date is None:
                    m = self._DATE_RE.search(loc)
                    if m:
                        post_date = datetime.date(
                            int(m.group(1)), int(m.group(2)), int(m.group(3))
                        )
                if post_date is None:
                    post_date = datetime.date.today()
                items.append((post_date, loc))
            items.sort(key=lambda x: x[0], reverse=True)
            return items if items else None
        except Exception:
            return None

    def _crawl_index(self, base_url: str) -> list[tuple[datetime.date, str]]:
        """Crawl the blog index pages to collect post URLs when no sitemap exists."""
        session = self._session()
        seen: set[str] = set()
        page = 1

        while True:
            # Try both /page/N/ and /pageN/ pagination styles
            if page == 1:
                url = base_url
            else:
                url = f"{base_url.rstrip('/')}/page/{page}/"
            r = session.get(url, timeout=20)
            if r.status_code == 404 and page > 1:
                # Try alternate pagination style: /page2/
                url = f"{base_url.rstrip('/')}/page{page}/"
                r = session.get(url, timeout=20)
            if r.status_code == 404:
                break
            r.raise_for_status()
            soup = BeautifulSoup(r.text, "html.parser")

            found_new = False
            # Prefer post links from headings (more reliable than all <a> tags)
            candidates = soup.select("h1 a[href], h2 a[href], h3 a[href]")
            if not candidates:
                candidates = soup.find_all("a", href=True)
            for a in candidates:
                abs_url = urljoin(base_url, a["href"]).rstrip("/")
                # Must be on same host, not a skip segment, not base itself
                if not abs_url.startswith(base_url.rstrip("/")):
                    continue
                path = abs_url.replace(base_url.rstrip("/"), "")
                if any(path.startswith(s) for s in self._SKIP_SEGMENTS):
                    continue
                if abs_url == base_url.rstrip("/"):
                    continue
                if abs_url not in seen:
                    seen.add(abs_url)
                    found_new = True

            if not found_new:
                break
            page += 1

        items: list[tuple[datetime.date, str]] = []
        for u in seen:
            m = self._DATE_RE.search(u)
            post_date = (
                datetime.date(int(m.group(1)), int(m.group(2)), int(m.group(3)))
                if m
                else datetime.date.today()
            )
            items.append((post_date, u))
        items.sort(key=lambda x: x[0], reverse=True)
        return items

    def _collect_post_urls(self, base_url: str) -> list[tuple[datetime.date, str]]:
        """Try sitemap first, fall back to index crawl."""
        result = self._try_sitemap(base_url)
        if result is not None:
            return result
        return self._crawl_index(base_url)

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

        for post_date, post_url in self._collect_post_urls(self.base_url):
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

    fetchers = [
        FetchGithubBlogs(name="aipiano", base_url="https://aipiano.github.io"),
        FetchGithubBlogs(name="udohsolomon", base_url="https://udohsolomon.github.io"),
        FetchGithubBlogs(name="adaning", base_url="https://adaning.github.io"),
    ]
    for fetcher in fetchers:
        items = fetcher.fetch()
        print(f"Fetched {len(items)} items from {fetcher.name}")
        db_mgr = DatabaseManager("test_github_blogs.db")
        for item in items:
            if not db_mgr.entry_exists(item["link"]):
                print(f"New post: {item['title']}")
                db_mgr.mark_as_seen(item["link"])
                break
