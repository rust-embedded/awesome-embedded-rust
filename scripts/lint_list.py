#!/usr/bin/env python3
"""
Awesome Embedded Rust List Linter

Validates the curated list format, checks entry consistency, verifies
alphabetical ordering, detects duplicates, and validates link/badge URLs.
"""

import argparse
import re
import sys
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple
from urllib.parse import urlparse


class Severity(Enum):
    ERROR = "ERROR"
    WARNING = "WARNING"
    INFO = "INFO"


@dataclass
class LintIssue:
    severity: Severity
    line_number: int
    message: str
    line_content: str = ""
    rule: str = ""


@dataclass
class LintReport:
    filepath: str
    issues: List[LintIssue] = field(default_factory=list)
    total_entries: int = 0
    total_categories: int = 0
    total_links: int = 0

    @property
    def error_count(self) -> int:
        return sum(1 for i in self.issues if i.severity == Severity.ERROR)

    @property
    def warning_count(self) -> int:
        return sum(1 for i in self.issues if i.severity == Severity.WARNING)


# ── Regex Patterns ──────────────────────────────────────────────────────────

# Matches list entries like: - [Name](url) - Description
ENTRY_RE = re.compile(
    r"^[-*]\s+\[([^\]]+)\]\(([^)]+)\)\s*[-–—:]?\s*(.*)"
)
# Matches headings
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*)")
# Matches shield.io badge URLs
SHIELD_RE = re.compile(
    r"https?://img\.shields\.io/[^\s\)]+", re.IGNORECASE
)
# Matches crates.io URLs
CRATES_RE = re.compile(r"https?://crates\.io/crates/[\w-]+")
# Matches GitHub URLs
GITHUB_RE = re.compile(r"https?://github\.com/[\w.-]+/[\w.-]+")
# Matches docs.rs URLs
DOCSRS_RE = re.compile(r"https?://docs\.rs/[\w-]+")


# ── Entry Format Validation ────────────────────────────────────────────────

def validate_entry_format(line: str, line_num: int) -> List[LintIssue]:
    """Validate the format of a list entry."""
    issues = []
    match = ENTRY_RE.match(line.strip())

    if not match:
        # It might be a sub-item or non-entry line
        stripped = line.strip()
        if stripped.startswith(("-", "*")) and "[" in stripped:
            issues.append(
                LintIssue(
                    severity=Severity.WARNING,
                    line_number=line_num,
                    message="Entry appears malformed — expected format: - [Name](url) - Description",
                    line_content=stripped,
                    rule="entry-format",
                )
            )
        return issues

    name, url, description = match.group(1), match.group(2), match.group(3)

    # Check name is not empty
    if not name.strip():
        issues.append(
            LintIssue(
                severity=Severity.ERROR,
                line_number=line_num,
                message="Entry has empty name",
                line_content=line.strip(),
                rule="entry-name",
            )
        )

    # Check URL validity
    if not _is_valid_url(url):
        issues.append(
            LintIssue(
                severity=Severity.ERROR,
                line_number=line_num,
                message=f"Invalid URL: {url}",
                line_content=line.strip(),
                rule="entry-url",
            )
        )

    # Check description exists
    if not description.strip():
        issues.append(
            LintIssue(
                severity=Severity.INFO,
                line_number=line_num,
                message=f"Entry '{name}' has no description",
                line_content=line.strip(),
                rule="entry-description",
            )
        )

    # Check description doesn't start with lowercase (style)
    if description.strip() and description.strip()[0].islower():
        issues.append(
            LintIssue(
                severity=Severity.INFO,
                line_number=line_num,
                message=f"Description for '{name}' starts with lowercase",
                line_content=line.strip(),
                rule="entry-style",
            )
        )

    return issues


def _is_valid_url(url: str) -> bool:
    """Check if URL is valid."""
    try:
        parsed = urlparse(url)
        return parsed.scheme in ("http", "https") and bool(parsed.netloc)
    except ValueError:
        return False


# ── Category Consistency ────────────────────────────────────────────────────

def check_category_consistency(lines: List[str]) -> List[LintIssue]:
    """Check heading hierarchy and category consistency."""
    issues = []
    heading_levels = []

    for i, line in enumerate(lines, start=1):
        match = HEADING_RE.match(line)
        if match:
            level = len(match.group(1))
            title = match.group(2).strip()
            heading_levels.append((i, level, title))

    # Check for heading level jumps (e.g., ## followed by ####)
    for idx in range(1, len(heading_levels)):
        prev_line, prev_level, _prev_title = heading_levels[idx - 1]
        curr_line, curr_level, curr_title = heading_levels[idx]
        if curr_level > prev_level + 1:
            issues.append(
                LintIssue(
                    severity=Severity.WARNING,
                    line_number=curr_line,
                    message=(
                        f"Heading level jumps from h{prev_level} (line {prev_line}) "
                        f"to h{curr_level} for '{curr_title}'"
                    ),
                    rule="heading-hierarchy",
                )
            )

    return issues


# ── Alphabetical Ordering ──────────────────────────────────────────────────

def check_alphabetical_order(lines: List[str]) -> List[LintIssue]:
    """Check that entries within each section are alphabetically ordered."""
    issues = []
    current_section = ""
    section_entries: List[Tuple[int, str]] = []

    def _check_section(entries: List[Tuple[int, str]], section: str):
        for i in range(1, len(entries)):
            prev_name = entries[i - 1][1].lower()
            curr_name = entries[i][1].lower()
            if curr_name < prev_name:
                issues.append(
                    LintIssue(
                        severity=Severity.WARNING,
                        line_number=entries[i][0],
                        message=(
                            f"'{entries[i][1]}' should come before "
                            f"'{entries[i-1][1]}' in section '{section}'"
                        ),
                        rule="alphabetical-order",
                    )
                )

    for i, line in enumerate(lines, start=1):
        heading_match = HEADING_RE.match(line)
        if heading_match:
            if section_entries:
                _check_section(section_entries, current_section)
            current_section = heading_match.group(2).strip()
            section_entries = []
            continue

        entry_match = ENTRY_RE.match(line.strip())
        if entry_match:
            section_entries.append((i, entry_match.group(1)))

    # Check final section
    if section_entries:
        _check_section(section_entries, current_section)

    return issues


# ── Duplicate Detection ────────────────────────────────────────────────────

def detect_duplicates(lines: List[str]) -> List[LintIssue]:
    """Detect duplicate entries by name or URL."""
    issues = []
    seen_names: Dict[str, int] = {}
    seen_urls: Dict[str, int] = {}

    for i, line in enumerate(lines, start=1):
        match = ENTRY_RE.match(line.strip())
        if not match:
            continue

        name = match.group(1).strip().lower()
        url = match.group(2).strip().lower()

        if name in seen_names:
            issues.append(
                LintIssue(
                    severity=Severity.ERROR,
                    line_number=i,
                    message=f"Duplicate entry name '{match.group(1)}' (first at line {seen_names[name]})",
                    line_content=line.strip(),
                    rule="duplicate-name",
                )
            )
        else:
            seen_names[name] = i

        if url in seen_urls:
            issues.append(
                LintIssue(
                    severity=Severity.WARNING,
                    line_number=i,
                    message=f"Duplicate URL (first at line {seen_urls[url]})",
                    line_content=line.strip(),
                    rule="duplicate-url",
                )
            )
        else:
            seen_urls[url] = i

    return issues


# ── Badge/Shield.io Validation ─────────────────────────────────────────────

def validate_badges(content: str) -> List[LintIssue]:
    """Validate shield.io badge URLs in the content."""
    issues = []
    for i, line in enumerate(content.split("\n"), start=1):
        for match in SHIELD_RE.finditer(line):
            badge_url = match.group(0)
            # Check badge URL structure
            parsed = urlparse(badge_url)
            path = parsed.path
            if not path or path == "/":
                issues.append(
                    LintIssue(
                        severity=Severity.WARNING,
                        line_number=i,
                        message=f"Empty shield.io badge path: {badge_url}",
                        rule="badge-format",
                    )
                )
            # Check for deprecated badge formats
            if "/badge/" not in path and "/static/" not in path and "?" not in badge_url:
                if path.count("-") < 1 and path.count("/") < 3:
                    issues.append(
                        LintIssue(
                            severity=Severity.INFO,
                            line_number=i,
                            message=f"Badge URL may use deprecated format: {badge_url}",
                            rule="badge-deprecated",
                        )
                    )

    return issues


# ── Link Format Validation ─────────────────────────────────────────────────

def validate_links(lines: List[str]) -> List[LintIssue]:
    """Validate link formats in entries."""
    issues = []
    for i, line in enumerate(lines, start=1):
        match = ENTRY_RE.match(line.strip())
        if not match:
            continue

        url = match.group(2).strip()

        # Prefer HTTPS
        if url.startswith("http://"):
            issues.append(
                LintIssue(
                    severity=Severity.WARNING,
                    line_number=i,
                    message=f"Consider using HTTPS instead of HTTP: {url}",
                    rule="prefer-https",
                )
            )

        # Check for trailing slashes inconsistency in GitHub URLs
        github_match = GITHUB_RE.match(url)
        if github_match and url.endswith("/"):
            issues.append(
                LintIssue(
                    severity=Severity.INFO,
                    line_number=i,
                    message="GitHub URL has trailing slash — consider removing for consistency",
                    rule="trailing-slash",
                )
            )

    return issues


# ── Main Linter ────────────────────────────────────────────────────────────

def lint_file(filepath: str) -> LintReport:
    """Run all lint checks on a file."""
    report = LintReport(filepath=filepath)

    try:
        content = Path(filepath).read_text(encoding="utf-8", errors="replace")
    except OSError as e:
        report.issues.append(
            LintIssue(
                severity=Severity.ERROR,
                line_number=0,
                message=f"Cannot read file: {e}",
                rule="file-read",
            )
        )
        return report

    lines = content.split("\n")

    # Count entries and categories
    for line in lines:
        if ENTRY_RE.match(line.strip()):
            report.total_entries += 1
        if HEADING_RE.match(line):
            report.total_categories += 1

    report.total_links = report.total_entries

    # Run all checks
    for i, line in enumerate(lines, start=1):
        if line.strip().startswith(("-", "*")) and "[" in line:
            report.issues.extend(validate_entry_format(line, i))

    report.issues.extend(check_category_consistency(lines))
    report.issues.extend(check_alphabetical_order(lines))
    report.issues.extend(detect_duplicates(lines))
    report.issues.extend(validate_badges(content))
    report.issues.extend(validate_links(lines))

    return report


def format_report(report: LintReport, verbose: bool = False) -> str:
    """Format a lint report as text."""
    lines = []
    lines.append(f"\n{'='*60}")
    lines.append(f"  Lint Report: {report.filepath}")
    lines.append(f"{'='*60}")
    lines.append(f"  Entries: {report.total_entries}  |  Categories: {report.total_categories}")
    lines.append(f"  Errors: {report.error_count}  |  Warnings: {report.warning_count}")

    if report.issues:
        lines.append("")
        for issue in sorted(report.issues, key=lambda x: (x.severity.value, x.line_number)):
            prefix = f"  [{issue.severity.value}]"
            loc = f" L{issue.line_number}" if issue.line_number > 0 else ""
            rule_str = f" ({issue.rule})" if issue.rule else ""
            lines.append(f"{prefix}{loc}{rule_str}: {issue.message}")
            if verbose and issue.line_content:
                lines.append(f"         > {issue.line_content}")
    else:
        lines.append("\n  No issues found. ✓")

    lines.append(f"\n{'='*60}")
    return "\n".join(lines)


# ── CLI ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Lint awesome-embedded-rust list for formatting and consistency."
    )
    parser.add_argument(
        "files",
        nargs="*",
        default=["README.md"],
        help="Files to lint (default: README.md).",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show line content for each issue.",
    )
    parser.add_argument(
        "--errors-only",
        action="store_true",
        help="Only show errors, not warnings or info.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as errors (affects exit code).",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output report as JSON.",
    )

    args = parser.parse_args()

    all_reports = []
    for filepath in args.files:
        report = lint_file(filepath)
        if args.errors_only:
            report.issues = [i for i in report.issues if i.severity == Severity.ERROR]
        all_reports.append(report)

    if args.json:
        import json
        output = {
            "files": [
                {
                    "file": r.filepath,
                    "entries": r.total_entries,
                    "categories": r.total_categories,
                    "errors": r.error_count,
                    "warnings": r.warning_count,
                    "issues": [
                        {
                            "severity": i.severity.value,
                            "line": i.line_number,
                            "rule": i.rule,
                            "message": i.message,
                        }
                        for i in r.issues
                    ],
                }
                for r in all_reports
            ]
        }
        print(json.dumps(output, indent=2))
    else:
        for report in all_reports:
            print(format_report(report, verbose=args.verbose))

    total_errors = sum(r.error_count for r in all_reports)
    total_warnings = sum(r.warning_count for r in all_reports)

    if total_errors > 0 or (args.strict and total_warnings > 0):
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
