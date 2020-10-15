function OcMap = imageToOccupancy(im,varargin)
    
    resolution = varargin{1};
    
    switch nargin
        case 2
            im = imresize(im,resolution,'bilinear');
        case 3
            scale = varargin{2};
            im = imresize(im,scale*resolution,'bilinear');
        case 4
            width = varargin{2};
            height = varargin{3};
            
            im = imresize(im,[width height].*resolution,'bilinear');
    end
    
    OcMap = occupancyMap(im,resolution);
end
