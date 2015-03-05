classdef VarClass
%Took some code from http://people.math.sc.edu/howard/Classes/726/inmat.html
    properties
        val
        var
    end
    
    methods
        
        function obj = VarClass(val,var)
                
            %validateattributes(val,{'numeric'},{'scalar'});
            %validateattributes(var,{'numeric'},{'scalar','nonnegative'});

            obj.val = double(val);
            obj.var = double(var);
        end

        function out = plus(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end
            
            valOut = A.val + B.val;
            varOut = A.var + B.var;
            out = VarClass(valOut,varOut);
        end

        function out = minus(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            valOut = A.val - B.val;
            varOut = A.var + B.var;
            out = VarClass(valOut,varOut);
        end
        
        function out = uminus(A)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end

            out = VarClass(-A.val,A.var);
        end

        function out = times(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            valOut = A.val .* B.val;
            varOut = A.var .* B.var + (A.val.^2).*B.var + (B.val.^2).*A.var;
            out = VarClass(valOut,varOut);
        end      

        function out = mtimes(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            if (size(B,1) == size(A,2))
                out = repmat(VarClass(0,0),size(A,1),size(B,2));

                for i = 1:size(A,1)
                    for j = 1:size(B,2)
                        for k = 1:size(A,2)
                            out(i,j).val = A(i,k).val .* B.val(k,j);
                            out(i,j).var = A(i,k).var .* B.var(k,j) + (A(i,k).val.^2).*B(k,j).var + (B(k,j).val.^2).*A(i,k).var;
                        end
                    end
                end

            else
                error('The dimensions of the matrices must agree.')
            end
        end    

        function out = power(A,p)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            validateattributes(p,{'numeric'},{'scalar','positive','integer'})

            out = A;
            for i = 2:p
                out = out .* A;
            end
        end     

        function out = mpower(A,p)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            validateattributes(p,{'numeric'},{'scalar','positive','integer'})

            out = A;
            for i = 2:p
                out = out .* A;
            end
        end   

        function out = rdivide(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            tempVal = 1./B.val;
            tempVar = B.var./(B.val.^4);

            valOut = A.val .* tempVal;
            varOut = A.var .* tempVar + (A.val.^2).*tempVar + (tampVal.^2).*A.var;
            
            out = VarClass(valOut,varOut);
        end

        function out = mrdivide(A,B)

            if(isnumeric(A))
                A = VarClass(A,0);
            else
                validateattributes(A,{'VarClass'},{'scalar'});
            end
            if(isnumeric(B))
                B = VarClass(B,0);
            else
                validateattributes(B,{'VarClass'},{'scalar'});
            end

            tempVal = 1./B.val;
            tempVar = B.var./(B.val.^4);

            valOut = A.val .* tempVal;
            varOut = A.var .* tempVar + (A.val.^2).*tempVar + (tempVal.^2).*A.var;
            out = VarClass(valOut,varOut);
        end
        
        function out = inv(A)
        end
    end
end