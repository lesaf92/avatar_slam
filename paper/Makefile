# Build the manuscript: make (draft) | make check | make submission-check | make clean
LATEXMK ?= latexmk
OUT := build

.PHONY: all check submission-check clean

all:
	$(LATEXMK) -pdf -interaction=nonstopmode -halt-on-error -outdir=$(OUT) main.tex

check:
	python3 ../tools/check_citations.py

submission-check:
	python3 ../tools/check_citations.py --submission

clean:
	$(LATEXMK) -C -outdir=$(OUT) main.tex
	rm -rf $(OUT)
