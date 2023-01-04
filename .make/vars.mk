PROJECT_NAME = $(shell cat Cargo.toml | grep 'name =' | cut -d' ' -f3 | cut -d'"' -f2)

#
# Version info of this repo
#
ifneq (${CI_COMMIT_TAG},) ## GitLab CI
	GIT_COMMIT_TAG ?= ${CI_COMMIT_TAG}
else
	GIT_COMMIT_TAG ?= $(shell git describe --tags --abbrev=0 2>/dev/null)
endif
ifneq (${CI_COMMIT_BRANCH},) ## GitLab CI
	GIT_COMMIT_BRANCH ?= ${CI_COMMIT_BRANCH}
else
	GIT_COMMIT_BRANCH ?= $(shell git rev-parse --abbrev-ref HEAD)
endif
ifneq (${CI_COMMIT_SHORT_SHA},) ## GitLab CI
	GIT_COMMIT_HASH_SHORT ?= ${CI_COMMIT_SHORT_SHA}
else
	GIT_COMMIT_HASH_SHORT ?= $(shell git rev-parse --short HEAD)
endif
GIT_COMMITS_SINCE_LAST_TAG ?= $(shell git rev-list --count ${GIT_COMMIT_TAG}..HEAD)
ifeq ($(strip $(shell git status --porcelain 2>/dev/null)),)
	GIT_TREE_STATE?=clean
else
	GIT_TREE_STATE?=dirty
endif

#
# git-based version info
#
GIT_DERIVED_SEMVER_BASE ?= $(or ${GIT_COMMIT_TAG},0)-$(or ${GIT_COMMITS_SINCE_LAST_TAG},0)
GIT_DERIVED_SEMVER ?= ${GIT_DERIVED_SEMVER_BASE}
GIT_DERIVED_SEMVER_LONG ?= ${GIT_DERIVED_SEMVER_BASE}-$(or ${GIT_COMMIT_HASH_SHORT},000000)
ifeq ($(GIT_TREE_STATE),dirty)
  GIT_DERIVED_SEMVER := ${GIT_DERIVED_SEMVER}-dirty
  GIT_DERIVED_SEMVER_LONG := ${GIT_DERIVED_SEMVER_LONG}-dirty
endif
