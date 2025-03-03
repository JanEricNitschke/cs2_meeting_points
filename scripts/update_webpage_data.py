import json
import pathlib
from typing import TypedDict


class MapData(TypedDict):
    gif: str
    images: list[str]


GalleryData = dict[str, MapData]


def merge_json_files(directory: pathlib.Path) -> GalleryData:
    """Merge all JSON files in the given directory into a single dictionary."""
    merged_data = {}

    for file in directory.glob("*.json"):
        with file.open("r", encoding="utf-8") as f:
            data: GalleryData = json.load(f)
            merged_data.update(data)  # Modify as needed for proper merging

    return merged_data


def update_index_html(index_file: pathlib.Path, json_data: GalleryData) -> None:
    """Replace `__GALLERY_DATA__` in index.html with the merged JSON string."""
    json_string = json.dumps(json_data, indent=2)

    with index_file.open("r", encoding="utf-8") as f:
        content = f.read()

    content = content.replace("__GALLERY_DATA__", json_string)

    with index_file.open("w", encoding="utf-8") as f:
        f.write(content)


def main() -> None:
    json_dir = pathlib.Path("webpage_data")
    index_file = pathlib.Path("index.html")

    merged_data = merge_json_files(json_dir)
    update_index_html(index_file, merged_data)


if __name__ == "__main__":
    main()
