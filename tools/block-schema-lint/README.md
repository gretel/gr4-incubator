# block-schema-lint

Extract and validate GR4 block descriptors from C++ headers.

Output is markdown: block info in tables, validation in severity/file/message tables.

## Usage

### CLI

```bash
# Scan + validate (exit 1 on errors)
uv run tools/block-schema-lint/scan_blocks.py path/to/Block.hpp

# Scan without validation (just descriptor dump)
uv run tools/block-schema-lint/scan_blocks.py path/to/Block.hpp --no-validate

# JSON catalog export (also runs validation by default)
uv run tools/block-schema-lint/scan_blocks.py path/to/Block.hpp --json

# JSON export, no validation
uv run tools/block-schema-lint/scan_blocks.py path/to/Block.hpp --json --no-validate

# Full pipeline: export + Cue format check (implies --json, requires cue)
uv run tools/block-schema-lint/scan_blocks.py path/to/Block.hpp --cue
```

### CMake (standalone)

```bash
# Configure and build (from repo root)
cmake -B build -S tools/block-schema-lint
cmake --build build --target block-schema-lint
```

The tool auto-discovers gnuradio4 headers and incubator block headers relative to its source directory. Set ``-DGR4_SOURCE_DIR=/path/to/gnuradio4`` to override the default sibling lookup.

## Examples

```markdown
## `gr::incubator::basic::Copy` (Copy.hpp)

| Kind | Name | Type | Details |
|------|------|------|---------|
| port | `in` | `T` | dir=input |
| port | `out` | `T` | dir=output |

- **template_params**: `T`
- **type_expansions**: `uint8_t`, `int16_t`, `int32_t`
- **base_classes**: `Block<Copy<T>>`
```

With `--validate`, a validation summary table precedes the block descriptors:

```markdown
## Validation

8 warnings

| Severity | File | Message |
|----------|------|---------|
| WARNING | SigMfMetadata.hpp | No GR_REGISTER_BLOCK found in header |
```

## Exit codes

- 0: all checks passed
- 1: validation failures, parse errors, or bad file paths

## Development

### Setup

```bash
cd tools/block-schema-lint
uv sync --dev
```

Installs dev tools into `.venv/`:

- `basedpyright` — type checker

### Type checking

```bash
uv run basedpyright scan_blocks.py
```

The project defines TypedDicts (`MemberDict`, `BlockDict`, `IssueDict`, etc.)
instead of bare `dict[str, Any]` — real type errors (`reportMissingTypeArgument`,
`reportPossiblyUnboundVariable`) are caught at error level.

A minimal `typings/clang/cindex.pyi` stub provides type info for the
`clang.cindex` C extension (Cursor, Type, TranslationUnit, etc.).
Remaining warnings are all from clang.cindex dynamic attributes and are
suppressed to "warning" level.

### macOS notes

The tool prefers brew's LLVM over Xcode's clang to avoid libclang/resource-dir
version mismatch. It uses `-cxx-isystem /opt/homebrew/opt/llvm/include/c++/v1`
to find libc++ headers.

## Dependencies

- Python 3.11+, libclang (llvm), uv
- [Cue](https://cuelang.org/) (only for `--cue`)
