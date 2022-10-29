classdef bars < matlab.mixin.SetGet
   %Acts like a container
    properties
        count
        lengths
        diameters
        densitys
        masses
        inertias_x
        inertias_y
        inertias_z
        from_to
        from_to_extended

        mid_points
        alpha
        beta
        rotation_matrixes
    end
    
    methods
        function obj = bars()
        end
    end
end

