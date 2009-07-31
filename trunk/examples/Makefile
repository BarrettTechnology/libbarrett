
# List of all examples to make
EXAMPLES = ex1-jointposition ex2-gcomp ex3-datalogging \
           ex5-simplemove \
           example-cpp \
           cylinder mastermaster mastermaster_loc trimesh


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
