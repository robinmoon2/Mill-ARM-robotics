function prodline
%% A function named 'prodline' used to generate the working environment for both robots %%
% INPUT(S):  NIL
% OUTPUT(S): NIL
% Copyrights: Swaminath Venkateswaran, Associate Professor/Researcher: ESILV/DVRC
%%
    Z= -60;
    fill3([15,15,45,45],[-100,-100,-100,-100],[Z,20,20,Z],'c')
    hold on;
    fill3([15,15,15,15],[-100,100,100,-100],[Z,Z,20,20],'c')
    fill3([45,45,45,45],[-100,100,100,-100],[Z,Z,20,20],'c')
    fill3([15,15,45,45],[-100,100,100,-100],[20,20,20,20],'c')
    fill3([15,15,45,45],[100,100,100,100],[Z,20,20,Z],'c')
    fill3([15,15,45,45],[-100,100,100,-100],[Z,Z,Z,Z],'c')
    
    couleur= [0.80,0.80,0.80];
    fill3([-10,10,10,-10],[-10,-10,-10,-10],[0,0,Z,Z],couleur)
    fill3([-10,10,10,-10],[10,10,10,10],[0,0,Z,Z],couleur)
    fill3([-10,-10,-10,-10],[-10,-10,10,10],[0,Z,Z,0],couleur)
    fill3([10,10,10,10],[-10,-10,10,10],[0,Z,Z,0],couleur)
    fill3([-10,-10,10,10],[-10,10,10,-10],[0,0,0,0],couleur)
    fill3([-10,-10,10,10],[-10,10,10,-10],[Z,Z,Z,Z],couleur)
    
    xlim([-100 180])
    ylim([-100 180])
    zlim([Z 120])

end