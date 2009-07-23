
# List of all examples to make
EXAMPLES = cylinder draw draw2d draw2dq ex1-jointposition ex2-gcomp \
           mastermaster mastermaster_loc trimesh


# How to make each example

.PHONY : all
all : ${EXAMPLES}

# Rule to make each example
.PHONY : ${EXAMPLES}
${EXAMPLES}:
	${MAKE} -C $@


# How to clean each example

CLEANTARGS = ${addprefix clean/,${EXAMPLES}}

.PHONY : clean ${CLEANTARGS}
clean : ${CLEANTARGS}

${CLEANTARGS}:
	${MAKE} -C ${notdir $@} clean
