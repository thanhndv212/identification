function s_data = BT_getRecData(x_varName, x_index, x_record)
% function s_data = bt_getData(x_varName, x_index, x_record)
% Cette fonction permet d'extraire la valeur d'une variable d'un enregistrement
% paramètres:
%   x_varName: nom de la variable (chaine de caractères)
%   x_index: indice de la variable. Si variable non indicée, utiliser -1
%               On peut spécifier plusieurs indices, par exemple: 
%                  >> data = bt_readrecdata('record.bin');
%                  >> posCmd = bt_getRecData('jntCmdPos', [0:5], data)
%   x_record: structure d'enregistrement créée par la fonction bt_readrecdata
%   s_data: vecteur contenant les valeurs de la variable 
%
%see also BT_getRecData, BT_readRecData, BT_getRecFreq, bt_getRecVarList, bt_plotSignals

s_data = [];
for l_index = x_index;
    s_data = [s_data, getOneVariable(x_varName, l_index, x_record)];
end;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function s_data = getOneVariable(x_varName, x_index, x_record)
%% for each variable
l_index = 1;
l_found = 0;
l_nbVar = size(x_record.m_var,2);
while ( (l_found == 0) & (l_index <= l_nbVar))
    % if the name corresponds to the searched name
    if strcmp(x_record.m_var(l_index).m_name, x_varName) == 1
       % if format is after 2
       if x_record.m_format >= 2
          % if the index is OK
          if (x_record.m_var(l_index).m_index == x_index)
             l_found = 1;
          else
             l_index = l_index + 1;
          end;
       else
          l_found = 1;
       end;
    else
        l_index = l_index + 1;
    end
end

if l_found == 1
    s_data = x_record.m_var(l_index).m_value;
else
    s_data = [];
end