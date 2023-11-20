% A列 Time, B列 θのCSVを読み込み、
% Arduinoスケッチで使えるヘッダーファイルに変換するコード。

% θ1.csvの読み込み
data1 = readtable('linkAngle.csv', 'VariableNamingRule', 'preserve');
%linkValues = data1.("ƒÆ1");
linkValues = data1.("x");

% θ2.csvの読み込み
data2 = readtable('beltAngle.csv', 'VariableNamingRule', 'preserve');
%beltValues = data2.("ƒ¦2["x]");
beltValues = data2.("θ2");

% ヘッダーファイルの内容の生成
headerContent = "const float LinktgtAngle[" + length(linkValues) + "] = {";
for i = 1:length(linkValues)
    headerContent = headerContent + linkValues(i)*pi()/180 + ", ";
    if mod(i, 10) == 0
        headerContent = headerContent + newline;
    end
end
headerContent = headerContent + "};" + newline + newline;

headerContent = headerContent + "const float BelttgtAngle[" + length(beltValues) + "] = {";
for i = 1:length(beltValues)
    headerContent = headerContent + (beltValues(i)+90)*pi()/180 + ", ";
    if mod(i, 10) == 0
        headerContent = headerContent + newline;
    end
end
headerContent = headerContent + "};";

% ヘッダーファイルの書き出し
fid = fopen('DualMotorPID_Trajectory.h', 'wt');
fprintf(fid, headerContent);
fclose(fid);
