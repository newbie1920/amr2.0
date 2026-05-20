from __future__ import annotations

import re
from pathlib import Path

import pdfplumber


DOCS_DIR = Path(__file__).resolve().parents[1] / "docs"
PDF_FILES = [
    DOCS_DIR / "ADAPTIVE_PID_RESEARCH_PAPER.pdf",
    DOCS_DIR / "Feedback_linearized_trajectory_tracking.pdf",
    DOCS_DIR / "KinoDynamics_IEEE_Access.pdf",
]

LIGATURES = {
    "\ufb00": "ff",
    "\ufb01": "fi",
    "\ufb02": "fl",
    "\ufb03": "ffi",
    "\ufb04": "ffl",
    "\ufb05": "ft",
    "\ufb06": "st",
}


def normalize_text(text: str) -> str:
    for source, target in LIGATURES.items():
        text = text.replace(source, target)

    text = re.sub(r"\(cid:\d+\)", "", text)
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    text = re.sub(r"([A-Za-z])-\n([a-z])", r"\1\2", text)
    text = re.sub(r"\n{3,}", "\n\n", text)
    text = "\n".join(line.rstrip() for line in text.splitlines())
    return text.strip()


def extract_pdf_text(pdf_path: Path) -> str:
    parts: list[str] = []

    with pdfplumber.open(pdf_path) as pdf:
        for index, page in enumerate(pdf.pages, start=1):
            page_text = page.extract_text(x_tolerance=1.5, y_tolerance=3) or ""
            page_text = normalize_text(page_text)
            if not page_text:
                continue
            parts.append(f"## Page {index}\n\n{page_text}")

    return "\n\n".join(parts).strip()


def write_markdown(pdf_path: Path) -> Path:
    markdown_path = pdf_path.with_suffix(".md")
    title = pdf_path.stem.replace("_", " ").strip()
    body = extract_pdf_text(pdf_path)
    content = f"# {title}\n\nSource PDF: `{pdf_path.name}`\n\n{body}\n"
    markdown_path.write_text(content, encoding="utf-8")
    return markdown_path


def main() -> None:
    for pdf_path in PDF_FILES:
        markdown_path = write_markdown(pdf_path)
        print(f"Created {markdown_path}")


if __name__ == "__main__":
    main()
