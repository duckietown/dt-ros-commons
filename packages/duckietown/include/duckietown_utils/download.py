import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional

from .friendly_path_imp import friendly_path
from .locate_files_impl import locate_files
from .logging_logger import logger
from .memoization import memoize_simple
from .mkdirs import d8n_make_sure_dir_exists
from .path_utils import get_ros_package_path
from .paths import get_duckietown_cache_dir
from .test_hash import get_md5, parse_hash_url
from .yaml_pretty import yaml_load_plain


@memoize_simple
def get_dropbox_urls():
    logger.info("Getting urls...")
    sources = []
    # from .paths import get_duckiefleet_root, get_duckietown_root, \
    #    get_duckietown_data_dirs
    #    sources.append(get_duckiefleet_root())
    #    sources.append(get_duckietown_root())
    #    sources.extend(get_duckietown_data_dirs())

    import imp

    dirname = imp.find_module("easy_logs")[1]
    sources.append(dirname)

    try:
        sources.append(get_ros_package_path("easy_logs"))
    except:
        pass  # XXX

    sources = set(sources)

    found = []
    urls = {}
    for s in sources:
        pattern = "*.urls.yaml"

        filenames = locate_files(s, pattern, case_sensitive=False)
        for f in filenames:
            found.append(f)
            logger.debug(f"loading {f}")
            with open(f) as _:
                data = _.read()
            logger.info(data)
            f_urls = yaml_load_plain(data)
            for k, v in list(f_urls.items()):
                urls[k] = v

    msg = "Found %d urls in %s files:\n" % (len(urls), len(found))
    msg += "\n".join(found)
    if len(urls) < 10:
        msg += "\n\n the urls: {urls}"
    logger.info(msg)
    logger.info(repr(urls))

    def sanitize(url: str) -> str:
        if url.endswith("?dl=0"):  # here
            url = url.replace("?dl=0", "?dl=1")
        return url

    return dict([(k, sanitize(url)) for k, url in (urls.items())])


def download_if_not_exist(url: str, filename: str):
    if not os.path.exists(filename):
        logger.info("Path does not exist: %s" % filename)
        download_url_to_file(url, filename)
        if not os.path.exists(filename):
            msg = "I expected download_url_to_file() to raise an error if failed."
            msg += "\n url: %s" % url
            msg += "\n filename: %s" % filename
            raise AssertionError(msg)
    return filename


start_time = None
last_time = None


def reporthook(count, block_size, total_size):
    now = time.time()
    global start_time
    global last_time
    if count == 0:
        start_time = now
        last_time = start_time - 10
        return
    interval = now - last_time

    def in_MB(x):
        return x / (1024.0 * 1024.0)

    duration = now - start_time
    progress_size = int(count * block_size)
    speed = progress_size / (1024 * 1024 * duration)
    percent = count * block_size * 100 / total_size

    if percent != 100 and interval < 5:
        return

    sys.stderr.write(
        "downloaded %.2f MB of %.1fMB (%.1f%%) at %.2f MB/s in %1.f s\n"
        % (in_MB(progress_size), in_MB(total_size), percent, speed, duration)
    )

    # sys.stdout.write("\r...%d%%, %d MB, %d KB/s, %d seconds passed" %
    #                  (percent, progress_size / (1024 * 1024), speed, duration))
    sys.stdout.flush()

    last_time = now


def download_url_to_file(url, filename):
    logger.info("Download from %s" % url)
    tmp = filename + ".tmp_download_file"
    urllib.request.urlretrieve(url, tmp, reporthook)
    if not os.path.exists(filename):
        os.rename(tmp, filename)

    logger.info("-> %s" % friendly_path(filename))


def get_file_from_url(url: str) -> str:
    """
        Returns a local filename corresponding to the contents of the URL.
        The data is cached in caches/downloads/
    """
    basename = get_md5(url)
    if "jpg" in url:
        basename += ".jpg"

    cachedir = get_duckietown_cache_dir()
    filename = os.path.join(cachedir, basename)
    download_if_not_exist(url, filename)
    return filename


@memoize_simple
def get_sha12url():
    sha12url = {}
    urls = get_dropbox_urls()
    for u, v in list(urls.items()):
        u = str(u)  # .encode('utf-8')
        if u.startswith("hash:"):
            parsed = parse_hash_url(u)
            sha12url[parsed.sha1] = v
    return sha12url


def require_resource(basename: str, destination: Optional[str] = None) -> str:
    """ Basename: a file name how it is in urls.yaml

        If destination is None, it is saved in the global cache dir.

        It returns the destination file.
    """
    urls = get_dropbox_urls()
    if not basename in urls:
        msg = "No URL found for %r." % basename
        raise Exception(msg)
    else:
        url = urls[basename]
        if destination is None:
            dirname = get_duckietown_cache_dir()
            destination = os.path.join(dirname, basename)
        d8n_make_sure_dir_exists(destination)
        download_if_not_exist(url, destination)
        return destination
