function [T1_RPR, P2_RPR, T3_RPR] = InverseKinematicsRPR(Px, Py, Pz, R1_const, R2_const, R3_const, D1_const, D3_const)

    arg_asin = (R1_const - Pz) / D3_const;
   
    T3_sol1 = asin(arg_asin);
    % T3_sol2 = pi - T3_sol1; % Second solution
    
    T3_RPR = T3_sol1;
    c3 = cos(T3_RPR);

    A = D1_const + D3_const * c3;
    B_squared = Px^2 + Py^2 - A^2;
    B = sqrt(B_squared);
    
    P2_RPR = B - R2_const - R3_const;
    T1_RPR = atan2(Py, Px) - atan2(B, A);

end
