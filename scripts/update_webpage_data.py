"""Script to merge JSON files and update index.html with the merged data."""

import json
import pathlib
import re
from typing import TypedDict


class MapData(TypedDict):
    gif: str
    images: list[str]


GalleryData = dict[str, MapData]


def merge_json_files(directory: pathlib.Path) -> GalleryData:
    """Merge all JSON files in the given directory into a single dictionary."""
    merged_data: GalleryData = {}

    for file in directory.glob("*.json"):
        print(f"Merging file: {file}")
        with file.open("r", encoding="utf-8") as f:
            data: GalleryData = json.load(f)
            merged_data.update(data)

    return merged_data


def update_index_html(index_file: pathlib.Path, json_data: GalleryData) -> None:
    """Replace `__GALLERY_DATA__` in index.html with the merged JSON string."""


    with index_file.open("r", encoding="utf-8") as f:
        content = f.read()

    # Case 1: Placeholder exists, replace it directly
    if "__GALLERY_DATA__" in content:
        json_string = json.dumps(json_data, indent=2, sort_keys=True)
        content = content.replace("__GALLERY_DATA__", json_string)
    else:
        # Case 2: Extract existing JSON and update it
        match = re.search(r"const galleryData = (\{.*\});", content, re.DOTALL)
        print(f"Match: {match}")
        if match:
            existing_json: GalleryData = json.loads(match.group(1))
            existing_json.update(json_data)
            updated_json_string = json.dumps(existing_json, indent=2, sort_keys=True)
            content = re.sub(
                r"const galleryData = \{.*\};", f"const galleryData = {updated_json_string};", content, flags=re.DOTALL
            )

    with index_file.open("w", encoding="utf-8") as f:
        f.write(content)


def main() -> None:
    json_dir = pathlib.Path("webpage_data")
    index_file = pathlib.Path("index.html")

    merged_data = merge_json_files(json_dir)
    update_index_html(index_file, merged_data)


if __name__ == "__main__":
    main()
