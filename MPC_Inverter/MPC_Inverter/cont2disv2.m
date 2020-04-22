function [Aq, Bq] = cont2disv2(Ts,Lfilter,Cfilter)
    % Input filter model
    
    A = [0 -1/Lfilter 0;1/Cfilter 0 -1/Cfilter;0 0 0];
    B = [1/Lfilter; 0;0];   
    % Discretization of the input filter model
    [Aq,Bq]=c2d(A,B,Ts);
end
