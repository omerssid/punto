# Vendored dependencies

## imgui/ — Dear ImGui v1.91.5
Upstream: https://github.com/ocornut/imgui (tag v1.91.5)
Vendored rather than submoduled so the repo builds with no network access.
`examples/`, `docs/` and `.git/` stripped; `backends/` keeps only what the
GLFW + OpenGL3 path needs.
