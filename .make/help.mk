##
### help
##

INCLUDEDIR=$(dir $(filter %/help.mk,$(MAKEFILE_LIST)))

## help displays a brief list of available make targets along with their inline comment text
# For the target and the help text to be displayed, it must match the format
# <targetname>: [dependency...] ## <helptext>
# <targetname> must not contain a space
.PHONY: help
help:: ## Use `make help` to print help
	@if [[ "${MAKECMDGOALS}" != "help" ]] && [[ "${MAKECMDGOALS}" != "" ]]; then \
		$(MAKE) --no-print-directory explain $(subst help,,${MAKECMDGOALS}); \
		exit 1; \
	fi
	@echo -e "\033[33m${PROJECT_NAME}\033[0m"
	@echo -e "v${GIT_DERIVED_SEMVER_LONG}"
	@bash ${INCLUDEDIR}/explain.sh "" ${MAKEFILE_LIST}

# unfortunately, this commented-out code generates ugly warning messages at the start
# less ugly is to exit with non-zero status in this rule
# # If the first target is "explain"
# ifeq (explain,$(firstword $(MAKECMDGOALS)))
#   # turn all other mentioned targets into no-ops
#   $(eval $(wordlist 2,$(words ${MAKECMDGOALS}),${MAKECMDGOALS}):;@:)
# endif

## Any target that is detectable by help (see `make explain help`) can also have a more elaborate
# comment above to explain its use in more detail.
# 
# An explanation comment is started with a double hash (##), and can be followed by
# lines starting with single hashes (#), double hashes (##), or the target itself.
# 
# Example:
# # this comment will not be included in 'make explain <targetname>'
# ## this is the first comment to be included
# # this one is included as well
# ## and this one too
# <targetname>: [dependency...] ## <helptext>
explain:: ## Use `make explain <target>` to print extra help if available
	@[[ "${MAKECMDGOALS}" == "explain" ]] && echo -e '\033[31mno command given to explain\033[0m (see `make help`)'; \
	i=0; for target in ${MAKECMDGOALS}; do \
		i=$$((i+1)) && [[ $$i -eq 1 ]] && continue; \
		bash ${INCLUDEDIR}/explain.sh $$target ${MAKEFILE_LIST} || exit 1; \
	done; \
	echo ""; \
	echo "exiting with exit code 1 to prevent execution of commands"; \
	exit 1