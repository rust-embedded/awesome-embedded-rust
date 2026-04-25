#!/usr/bin/env python3
"""Tests for the awesome-embedded-rust list linter."""

import os
import tempfile
import pytest

from lint_list import (
    LintIssue,
    LintReport,
    Severity,
    check_alphabetical_order,
    check_category_consistency,
    detect_duplicates,
    format_report,
    lint_file,
    validate_badges,
    validate_entry_format,
    validate_links,
)


class TestValidateEntryFormat:
    def test_valid_entry(self):
        issues = validate_entry_format(
            "- [Embassy](https://github.com/embassy-rs/embassy) - Async embedded framework",
            1,
        )
        assert len([i for i in issues if i.severity == Severity.ERROR]) == 0

    def test_empty_name(self):
        issues = validate_entry_format("- [](https://example.com) - Description", 1)
        assert any(i.rule == "entry-name" for i in issues)

    def test_invalid_url(self):
        issues = validate_entry_format("- [Test](not-a-url) - Description", 1)
        assert any(i.rule == "entry-url" for i in issues)

    def test_no_description(self):
        issues = validate_entry_format("- [Test](https://example.com)", 1)
        assert any(i.rule == "entry-description" for i in issues)

    def test_lowercase_description(self):
        issues = validate_entry_format(
            "- [Test](https://example.com) - lowercase description", 1
        )
        assert any(i.rule == "entry-style" for i in issues)

    def test_non_entry_line(self):
        issues = validate_entry_format("Just a plain text line", 1)
        assert len(issues) == 0


class TestCategoryConsistency:
    def test_valid_hierarchy(self):
        lines = ["# Title", "## Section", "### Subsection"]
        issues = check_category_consistency(lines)
        assert len(issues) == 0

    def test_heading_jump(self):
        lines = ["# Title", "#### Deep"]
        issues = check_category_consistency(lines)
        assert len(issues) == 1
        assert issues[0].rule == "heading-hierarchy"

    def test_single_heading(self):
        lines = ["## Section"]
        issues = check_category_consistency(lines)
        assert len(issues) == 0


class TestAlphabeticalOrder:
    def test_sorted_entries(self):
        lines = [
            "## Tools",
            "- [Alpha](https://a.com) - First",
            "- [Beta](https://b.com) - Second",
            "- [Charlie](https://c.com) - Third",
        ]
        issues = check_alphabetical_order(lines)
        assert len(issues) == 0

    def test_unsorted_entries(self):
        lines = [
            "## Tools",
            "- [Charlie](https://c.com) - Third",
            "- [Alpha](https://a.com) - First",
        ]
        issues = check_alphabetical_order(lines)
        assert len(issues) == 1
        assert issues[0].rule == "alphabetical-order"

    def test_separate_sections(self):
        lines = [
            "## Section A",
            "- [Zebra](https://z.com) - Z",
            "## Section B",
            "- [Alpha](https://a.com) - A",
        ]
        issues = check_alphabetical_order(lines)
        assert len(issues) == 0


class TestDetectDuplicates:
    def test_no_duplicates(self):
        lines = [
            "- [One](https://one.com) - First",
            "- [Two](https://two.com) - Second",
        ]
        issues = detect_duplicates(lines)
        assert len(issues) == 0

    def test_duplicate_name(self):
        lines = [
            "- [Same](https://one.com) - First",
            "- [Same](https://two.com) - Second",
        ]
        issues = detect_duplicates(lines)
        assert any(i.rule == "duplicate-name" for i in issues)

    def test_duplicate_url(self):
        lines = [
            "- [One](https://same.com) - First",
            "- [Two](https://same.com) - Second",
        ]
        issues = detect_duplicates(lines)
        assert any(i.rule == "duplicate-url" for i in issues)


class TestValidateBadges:
    def test_valid_badge(self):
        content = "![badge](https://img.shields.io/badge/rust-embedded-orange)"
        issues = validate_badges(content)
        errors = [i for i in issues if i.severity == Severity.ERROR]
        assert len(errors) == 0

    def test_empty_badge_path(self):
        content = "![badge](https://img.shields.io/)"
        issues = validate_badges(content)
        assert any(i.rule == "badge-format" for i in issues)


class TestValidateLinks:
    def test_https_link(self):
        lines = ["- [Test](https://example.com) - Description"]
        issues = validate_links(lines)
        assert len([i for i in issues if i.rule == "prefer-https"]) == 0

    def test_http_link_warned(self):
        lines = ["- [Test](http://example.com) - Description"]
        issues = validate_links(lines)
        assert any(i.rule == "prefer-https" for i in issues)

    def test_github_trailing_slash(self):
        lines = ["- [Test](https://github.com/user/repo/) - Description"]
        issues = validate_links(lines)
        assert any(i.rule == "trailing-slash" for i in issues)


class TestLintFile:
    def test_valid_file(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
            f.write("# Title\n\n## Section\n\n- [Alpha](https://a.com) - First\n")
            f.flush()
            report = lint_file(f.name)
        os.unlink(f.name)
        assert report.error_count == 0
        assert report.total_entries == 1

    def test_file_not_found(self):
        report = lint_file("/nonexistent/file.md")
        assert report.error_count > 0

    def test_counts(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".md", delete=False) as f:
            f.write(
                "# Title\n## Cat1\n- [A](https://a.com) - D\n"
                "## Cat2\n- [B](https://b.com) - D\n"
            )
            f.flush()
            report = lint_file(f.name)
        os.unlink(f.name)
        assert report.total_entries == 2
        assert report.total_categories == 3  # Title + Cat1 + Cat2


class TestFormatReport:
    def test_clean_report(self):
        report = LintReport(filepath="test.md", total_entries=5, total_categories=2)
        output = format_report(report)
        assert "No issues found" in output

    def test_report_with_issues(self):
        report = LintReport(filepath="test.md")
        report.issues.append(
            LintIssue(
                severity=Severity.ERROR,
                line_number=10,
                message="Test error",
                rule="test-rule",
            )
        )
        output = format_report(report, verbose=True)
        assert "ERROR" in output
        assert "test-rule" in output


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
