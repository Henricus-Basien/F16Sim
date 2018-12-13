%global NrStates A_lo B_lo C_lo D_lo e

for i = 1:NrStates
	% --- A ---
    for j = 1:NrStates
        if abs(A_lo(i,j))<e
            A_lo(i,j) = 0;
        end
    end
    % --- B ---
    for j = 1:NrInputs
        if abs(B_lo(i,j))<e
            B_lo(i,j) = 0;
        end
    end
    % --- C ---
    for j = 1:NrOutputs
    	if abs(C_lo(j,i))<e
            C_lo(j,i) = 0;
        end
    end
    % --- D ---
    for j = 1:NrInputs
        if abs(D_lo(i,j))<e
            D_lo(i,j) = 0;
        end
    end
end