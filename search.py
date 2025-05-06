from pronk import PRONK


for i in range(1):
    pronk = PRONK()

    pronk.Evolve()   

    pronk.Show_Best()
    cont_boolean = True
    while cont_boolean:
        cont = input("Play again? ")
        if cont.lower() == "no":
            break

        pronk.Show_Best()

