"""Check if the regeneration of the map is needed.

The regeneration is needed if the last updated time of CS2 is later than the last run time of the script.
Prints a `"true"` or `"false"` whether a regeneration is needed or not
to be captured and used for the GitHub Actions."""

import argparse
import re
import subprocess
from datetime import datetime, timezone

import vdf


def get_current_utc_time() -> datetime:
    return datetime.now(timezone.utc)


def read_last_run_time(time_file: str) -> str:
    with open(time_file) as f:
        return f.read().strip()


def write_last_run_time(time_file: str, utc_time: str) -> None:
    with open(time_file, "w") as f:
        f.write(utc_time)
        f.write("\n")


def needs_regeneration(last_run_time: datetime, last_update_time: datetime) -> bool:
    return last_run_time < last_update_time


def get_last_update_time() -> datetime:
    """Get the last updated time of CS2 via steamcmd.

    THe ID for CS2 is 730."""
    command = [
        "steamcmd",
        "+login",
        "anonymous",
        "+app_info_print",
        "730",
        "+logoff",
        "+quit",
    ]
    result = subprocess.check_output(command).decode("utf-8")  # noqa: S603
    json_start = result.find("730")
    json_end_index = result.rfind("}")
    vdf_data = result[json_start : json_end_index + 1]
    vdf_data = re.sub(r'^(?!\s*[{}]|.*".*").*$', "", vdf_data, flags=re.MULTILINE)
    parsed_data = vdf.loads(vdf_data)
    timeline_marker_updated = int(parsed_data["730"]["common"]["timeline_marker_updated"])
    return datetime.fromtimestamp(timeline_marker_updated, timezone.utc)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "time_file",
        nargs="?",
        default="last_run_time.txt",
        help="Path to the time file",
    )
    args = parser.parse_args()

    last_run_time = datetime.fromisoformat(read_last_run_time(args.time_file))
    last_update_time = get_last_update_time()

    if last_run_time and needs_regeneration(last_run_time, last_update_time):
        write_last_run_time(args.time_file, get_current_utc_time().isoformat())
        print("true")
    else:
        print("false")


if __name__ == "__main__":
    main()
