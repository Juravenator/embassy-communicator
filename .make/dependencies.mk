# rust binaries path
export PATH := $(HOME)/.cargo/bin:$(PATH)

## Check for dependencies and install if possible.
# For packages that are meant to be installed by
# the systems package manager (e.g. curl), the
# target will fail if they are not present on the system.
# 
# Rust and related packages will be installed if not present.
# The proper cargo toolchain will be installed and set.
toolchain: ## ensure build dependencies are present
	@bash .make/toolchain.sh