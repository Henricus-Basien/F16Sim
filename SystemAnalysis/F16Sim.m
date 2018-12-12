function [OUT] = F16Sim(xu)

global USE_SI_UNITS

if (USE_SI_UNITS == 1)
    OUT = feval('F16Sim_SI',xu);
else
    OUT = feval('F16Sim_IU',xu);%feval('nlplant',xu);
end

end