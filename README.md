This repository started as an experiment of the following ideas:

First, on the role of whitespace in readability of code, and whether too much
whitespace leads to a sense of code being readable when it should not. By
removing all non essential whitespace, can we still make the code readable and
where does that lead us?

Secondly, it is an experiment in embedding as much build/link information into
the source code as much as possible, through special preprocessor or commented
out tags.

Thirdly, vine_effect.cpp tries to show how to build a complex system out of
multiple independent pieces that are contributing to a whole.

TODO(nicolas): actually make vine_effect.cpp more associative. There is a lot
of interdependency created by those templates which isn't really adequate with
the C/C++ model of compilation. I can see it in how defining operator+ created
a build failure after win32 was included. See https://gist.github.com/uucidl/0874c6e1e25d0db43d232bbd2c1b0bda

