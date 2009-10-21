(TeX-add-style-hook "master"
 (lambda ()
    (LaTeX-add-labels
     "splitting")
    (TeX-run-style-hooks
     "datetime"
     "latex2e"
     "memoir10"
     "memoir"
     "10pt"
     "oneside"
     "a4paper"
     "final"
     "english"
     "env/packages"
     "env/forloop"
     "env/languages"
     "env/graphics"
     "env/math"
     "env/preamble")))

