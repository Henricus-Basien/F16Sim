% Mu-Analysis and Synthesis Toolbox.
%
%    It is good practice to press the Return key after having filled out
%    any white edit field in the graphical interface. This executes the
%    callback to the edit control. Otherwise you may have to click twice on
%    any action push button to make it react. This is because the first
%    click executes the callback to the edit control. 
%
%
%    Known Problems
%   
%    On page 7-9 of the manual the following line is incorrect.
%    >> delta = [1+delta1 0 ; 0 ; 1+delta2];
%    It should be like the following line.
%    >> delta = [1+delta1 0 ; 0 1+delta2];
%
%    There is a problem when printing figures from the toolbox. 
%    When the figure prints the text may not appear on the printout.
%    A workaround for this problem is to remove the file pltcols.m 
%    from mutools/subs and replace it with a file with the same name
%    which only contains the following line:
%    function    pltcols(x) 

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.
