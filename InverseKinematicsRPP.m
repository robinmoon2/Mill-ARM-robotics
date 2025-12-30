function[t1,p2,p3] = InverseKinematicsRPP(Px,Py,Pz, offset_p2_z, offset_p3_radial)
    % Px, Py, Pz: Coordonnées cartésiennes cibles de l'effecteur final.
    % offset_p2_z:  Somme des longueurs fixes qui contribuent à la hauteur Z de la base du P2_RPP
    % offset_p3_radial: Somme des longueurs fixes qui contribuent à la distance radiale

    t1 = atan2(-Px, Py); 

    % Correction pour p2: Ajustement du décalage en Z
    p2 = Pz - offset_p2_z; 

    r_target_radial = sqrt(Px^2 + Py^2);

    p3 = r_target_radial - offset_p3_radial; 
end
