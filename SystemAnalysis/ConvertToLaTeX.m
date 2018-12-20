%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% LaTeX
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function ConvertToLaTeX(A)
    precision   = 3; % [#Digits]
    latex_table = latex(vpa(sym(A),precision))
end