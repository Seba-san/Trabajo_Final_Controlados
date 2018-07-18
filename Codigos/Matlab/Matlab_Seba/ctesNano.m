function [sal]=ctesNano(var,str,ind1)


try
if str=='num'    
    sal=var.num{1}(ind1);
end
if str=='den'
        sal=var.den{1}(ind1);
end

catch
    sal=0;
end

end