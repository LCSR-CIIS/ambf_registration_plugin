import numpy as np
import click 


@click.command()
@click.option('--input', help='Path to input csv file.')
@click.option('--output', default= './data', help='Output path to csv file.')
@click.option('--mm2m', default= True, help='Whether to convert first three to m.')
def refractor_txt(input, output, mm2m):
    print("Loading: " + input)
    data = np.loadtxt(input, delimiter=',')
    
    ## If the data has only 7 column add pseudo timestep column 
    if data.shape[1] == 7:
        data = np.insert(data, 1, np.arange(data.shape[0], dtype=float) ,axis=1)

    # multiply index 1,2,3 by 0.001 to convert mm to m.
    if(mm2m):   
        data[:, 1:3] *= 0.001

    np.savetxt(output+input, delimiter=", ")

if __name__ == 'main':
    refractor_txt()