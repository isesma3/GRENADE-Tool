import pandas as pd
import numpy as np

df1 = pd.read_csv('Compiled1-250.csv')
df2 = pd.read_csv('Compiled251-255.csv')
df3 = pd.read_csv('Compiled256-999.csv')
df4 = pd.read_csv('Compiled1000-1999.csv')
df5 = pd.read_csv('Compiled2000-2999.csv')
df6 = pd.read_csv('Compiled3000-3999.csv')
df7 = pd.read_csv('Compiled4000-4999.csv')
df8 = pd.read_csv('Compiled5000-5999.csv')
df9 = pd.read_csv('Compiled6000-6999.csv')
df10 = pd.read_csv('Compiled7000-7999.csv')

# dffinal = pd.concat([df1,df2,df3,df4,df5,df6,df7,df8])
dffinal = pd.concat([df1,df2,df3,df4,df5,df6,df7,df8,df9,df10])
dffinal = dffinal.reset_index()
DoE = pd.read_csv('../Inp/DOE.csv')
Labels = pd.DataFrame(DoE.columns.tolist()+dffinal.columns.tolist(),columns=['Label']).transpose()
if list(dffinal['Case']) == list(DoE['CASE']):
    All_out = pd.concat([DoE,dffinal],axis=1)
    #All_out = DoE.merge(dffinal, how='outer')
All_out.to_csv('Compiled1-7999.csv')
