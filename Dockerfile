FROM gitlab/gitlab-runner:latest

# Configure timezone
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install required packages
RUN apt-get update
RUN apt-get install -y sudo
RUN apt-get install -y software-properties-common
RUN apt-get install -y libssl-dev
RUN apt-get install -y python3
RUN apt-get install -y xz-utils
RUN apt-get install -y build-essential
RUN apt-get install -y ninja-build 
RUN apt-get install -y ccache
RUN apt-get install -y clang-tidy
RUN apt-get install -y doxygen

# Fix version of cmake
RUN git clone --depth 1 --branch v3.28.0 https://gitlab.kitware.com/cmake/cmake.git
RUN cd cmake && ./bootstrap && make install

# Fix version of clang-format
RUN wget --no-check-certificate -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
RUN add-apt-repository 'deb http://apt.llvm.org/focal/ llvm-toolchain-focal-14 main'
RUN apt-get install -y clang-format-14
RUN ln -s /usr/bin/clang-format-14 /usr/bin/clang-format

# Fix version of cppcheck
RUN git clone --depth 1 --branch 2.7 https://github.com/danmar/cppcheck.git
RUN cd cppcheck && make install FILESDIR=/usr/share/cppcheck

# Setup Miosix
ADD https://miosix.org/toolchain/MiosixToolchainInstaller.run MiosixToolchainInstaller.run
RUN sh MiosixToolchainInstaller.run
