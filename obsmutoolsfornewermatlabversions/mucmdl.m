function mucmdl
%MUCMDL Set up Mu-Analysis and Synthesis commandline demos.

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.
 
labelList=str2mat( ...
    'Intro to data types', ...
    'Time-domain manipulations');
 
nameList=str2mat( ...
    'msdemo1', ...
    'msdemo2');
 
figureFlagList=[
        0
	0];

cmdlnwin(labelList,nameList,figureFlagList);
 
