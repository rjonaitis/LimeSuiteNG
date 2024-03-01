FROM gcc:13 AS build-stage

COPY install_dependencies.sh install_dependencies.sh

RUN apt update && \
    apt-get install -y lcov && \
    ./install_dependencies.sh -y

COPY external/ external/
COPY cmake/ cmake/
COPY udev-rules/ udev-rules/
COPY plugins/ plugins/
COPY Changelog.txt Changelog.txt
COPY CMakeLists.txt CMakeLists.txt
COPY src/ src/

RUN cmake . -B build -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=OFF
RUN make -C build --no-print-directory -j$(nproc) limesuiteng-test_coverage

FROM scratch AS export-stage
COPY --from=build-stage build/limesuiteng-test_coverage/ /
