%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% LaTeX
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

function ConvertToLaTeX(A)
    precision   = 6; % [#Digits]
    latex_table = latex(vpa(sym(A),precision))
end