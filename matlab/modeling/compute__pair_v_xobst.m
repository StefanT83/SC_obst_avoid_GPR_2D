function pair_v_xobst = compute__pair_v_xobst(v_range,xobst_range, surface, deltatstop_desired)
%Identifies the intersction between a surface and a horizontal plane at height deltastop_desired and stores this as a curve inside pair_v_xobst    
%Algo: create s_up_inters=s_up_inters(id_v_range,id_xobst_range), a surface having heights (0 and 1), the intersection of which indicate the separation between the areas where: case1) s_up is below deltat_stop_safe; case2) s_up is above deltat_stop_safe    
%then swipe along one dimension to find where the intersection with deltatstop_desired occurs, and mark that (v,xobst) point in pair_v_xobst = pair_v_xobst(nr_points,2);  

s_up_inters = nan(length(v_range),length(xobst_range)); %ini & malloc
id__pair_v_xobst = 0; %ini

for id_v_range=1:length(v_range)
    for id_xobst_range=1:length(xobst_range)
        %by default, assume case1) s_up is below deltat_stop_safe
        s_up_inters(id_v_range,id_xobst_range) = 1; %'1' corresponds to 'value high' 
        
        %check whether actually we are in case2 and consequently adjust s_up_inters(id_v_range,id_xobst_range)    
        s_up__id_v_range__id_xobst_range = surface(id_v_range,id_xobst_range);
        if s_up__id_v_range__id_xobst_range > deltatstop_desired 
            s_up_inters(id_v_range,id_xobst_range) = 0; %'0' corresponds to 'value low' 
        end %if
        %otherwise do nothing
    end %for id_xobst_range=

    %apply diff along one dimension
    s_up_inters(id_v_range,:) = [diff(mrv(s_up_inters(id_v_range,:))) 0];

    % find that value where s_up_inters changes sign
    idx__id_xobst_range = find( s_up_inters(id_v_range,:) ~= 0);
    if ~isempty(idx__id_xobst_range)
        %store
        id__pair_v_xobst = id__pair_v_xobst+1; 
        pair_v_xobst(id__pair_v_xobst,1) = v_range(id_v_range);
        pair_v_xobst(id__pair_v_xobst,2) = xobst_range(idx__id_xobst_range);
    end %if ~isempty(.)
end %for id_v_range=
