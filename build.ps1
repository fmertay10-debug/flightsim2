# Configure + build + test.
cmake -S . -B build -G "MinGW Makefiles"
if ($LASTEXITCODE -ne 0) { exit 1 }
cmake --build build -j
if ($LASTEXITCODE -ne 0) { exit 1 }
ctest --test-dir build --output-on-failure
