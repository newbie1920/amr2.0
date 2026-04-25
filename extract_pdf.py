import pymupdf4llm
from pathlib import Path

pdf_path = Path("c:/code2/AMR2.0/Trajectory_Tracking_Tutorial.pdf")
md_text = pymupdf4llm.to_markdown(pdf_path)

md_path = Path("c:/code2/AMR2.0/Trajectory_Tracking_Tutorial.md")
md_path.write_text(md_text, encoding="utf-8")
print(f"Successfully converted to {md_path}")
