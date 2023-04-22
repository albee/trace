%{
Plots a cube with orientation on the given fig.
Inputs:
* r0 : 3-element vector that defines the cube origin
* R0: 3x3 rotation matrix that defines cube orientation relative to
inertial, R_IB
* erase : erase the latest plot (not the whole figure)
%}
function [fig, plot_cube_handles] = plot_FK3(fig, plot_cube_handles, r0, R0, erase)
    figure(fig)
    
    if(erase)
        delete(plot_cube_handles)
    end

%     COM_I = Center_of_Mass(self.r0, self.rL, self.robot);
%     r_I = [self.r0, self.rJ];
% 
%     scatter3(r_I(1,:),r_I(2,:),r_I(3,:),'black','filled');  % plot joints            
%     plot3(r_I(1,:),r_I(2,:),r_I(3,:),'black','Linewidth',2);  % plot links

    CLR = [0, 0, 1];
    ALPHA = .8;
    l = .2;
    plot_cube_handles = plot_cube(r0, R0, l, CLR, ALPHA);
%     plot_cube([1.25; 1.25; 1.25], eye(3), .25,  [0, 0, 1], 1.0);

%     scatter3(COM_I(1,:),COM_I(2,:),COM_I(3,:),50,'bl','filled');  % plot COM
end