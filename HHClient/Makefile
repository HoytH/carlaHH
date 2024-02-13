CARLADIR=$(CURDIR)/../
BUILDDIR=$(CURDIR)/build
INSTALLDIR=$(CURDIR)/libcarla-install
TOOLCHAIN=$(CURDIR)/ToolChain.cmake


build_libcarla: $(TOOLCHAIN)
	@cd $(CARLADIR); make setup
	@mkdir -p $(BUILDDIR)
	$(call log,Compiling asdfsdafLibCarla.client...)
	@{ \
		cd $(BUILDDIR); \
		if [ ! -f "build.ninja" ]; then \
		cmake \
			-G "Ninja" \
			-DCMAKE_BUILD_TYPE=Client \
			-DLIBCARLA_BUILD_RELEASE=ON \
			-DLIBCARLA_BUILD_DEBUG=OFF \
			-DLIBCARLA_BUILD_TEST=OFF \
			-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN) \
			-DCMAKE_INSTALL_PREFIX=$(INSTALLDIR) \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
			$(CARLADIR); \
	  fi; \
		ninja; \
		ninja install | grep -v "Up-to-date:"; \
	}

$(TOOLCHAIN):
	@echo "set(CC, \"\$$ENV{UE4_ROOT}/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang\")" >> $(TOOLCHAIN)
	@echo "set(CXX, \"\$$ENV{UE4_ROOT}/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang++\")" >> $(TOOLCHAIN)
	@echo "set(CC, "$ENV{UE4_ROOT}/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v17_clang-10.0.1-centos7/x86_64-unknown-linux-gnu/bin/clang")"
	@echo "set(CMAKE_C_COMPILER \$${CC})" >> $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_COMPILER \$${CXX})" >> $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_FLAGS \"\$${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fPIC -O3 -DNDEBUG\" CACHE STRING \"\" FORCE)" >> $(TOOLCHAIN)