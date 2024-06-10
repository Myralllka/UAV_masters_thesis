import numpy as np
import matplotlib.pyplot as plt

st_1 = """
1,plkf,0.0,0.0,0.049191953223208025
1,plkft,0.0,0.0,0.09095915083355784
1,dkf,0.0,0.0,0.030833638335096607
1,dkft,0.0,0.0,0.10686972656955025
1,svd-static,0.0,0.0,1.1754631402162659
1,svd-dynamic,0.0,0.0,5.6422072541826225
1,plkf,0.5,0.0,0.9049220774073579
1,plkft,0.5,0.0,1.5483660610882015
1,dkf,0.5,0.0,0.5748760947696888
1,dkft,0.5,0.0,1.7154749485228198
1,svd-static,0.5,0.0,5.422835501200106
1,plkf,1.0,0.0,1.6430686975010085
1,plkft,1.0,0.0,1.5304863478683637
1,dkf,1.0,0.0,1.6719525647682694
1,dkft,1.0,0.0,2.656321104246621
1,svd-static,1.0,0.0,5.438847796022196
1,plkf,2.0,0.0,3.435863447071106
1,plkft,2.0,0.0,1.9938209723881994
1,dkf,2.0,0.0,2.639449665627666
1,dkft,2.0,0.0,1.8648771634846384
1,svd-static,2.0,0.0,5.438127580947203
1,plkf,0.0,0.0,0.049191953223208025
1,plkft,0.0,0.0,0.09095915083355784
1,dkf,0.0,0.0,0.030833638335096607
1,dkft,0.0,0.0,0.10686972656955025
1,svd-static,0.0,0.0,1.1754631402162659
1,plkf,0.0,3.0,2.951971106837336
1,plkft,0.0,3.0,2.602098302206092
1,dkf,0.0,3.0,3.13608554324884
1,dkft,0.0,3.0,3.0905351746620897
1,svd-static,0.0,3.0,1.8710696746948987
1,plkf,0.0,6.0,4.654665409671775
1,plkft,0.0,6.0,3.6054581638017438
1,dkf,0.0,6.0,5.203260828323245
1,dkft,0.0,6.0,4.219342645154801
1,svd-static,0.0,6.0,2.8603978766366644
1,plkf,0.0,9.0,5.316110459872037
1,plkft,0.0,9.0,4.130122513174789
1,dkf,0.0,9.0,5.880380983363308
1,dkft,0.0,9.0,4.8820686273175395
1,svd-static,0.0,9.0,3.684342652374838
"""
st_2 = """
2,svd-static,0.0,0.0,11.704021599296123
2,svd-dynamic,0.0,0.0,12.057077754981448
2,plkf,0.0,0.0,11.872050814588597
2,plkft,0.0,0.0,2.482267177414454
2,dkf,0.0,0.0,7.2616730883312
2,dkft,0.0,0.0,2.4408869419521255
2,plkf,0.0,3.0,12.601269297480123
2,plkft,0.0,3.0,2.182532877012504
2,dkf,0.0,3.0,10.215029011248008
2,dkft,0.0,3.0,6.99415447044583
2,plkf,0.0,6.0,11.085077967074977
2,plkft,0.0,6.0,3.3151405024070653
2,dkf,0.0,6.0,10.3593775329148
2,dkft,0.0,6.0,9.325385391981037
2,plkf,0.0,9.0,10.407145709708598
2,plkft,0.0,9.0,4.57371226781136
2,dkf,0.0,9.0,10.941080747289318
2,dkft,0.0,9.0,9.978915328056061
2,plkf,0.0,0.0,11.872050814588597
2,plkft,0.0,0.0,2.482267177414454
2,dkf,0.0,0.0,7.2616730883312
2,dkft,0.0,0.0,2.4408869419521255
2,plkf,0.5,0.0,13.383719815626888
2,plkft,0.5,0.0,1.584508931839022
2,dkf,0.5,0.0,13.132161035675622
2,dkft,0.5,0.0,1.7916580375570643
2,plkf,1.0,0.0,13.540728584993506
2,plkft,1.0,0.0,1.579281527836601
2,dkf,1.0,0.0,11.333424073753484
2,dkft,1.0,0.0,1.2980546633151595
2,plkf,2.0,0.0,13.77099685659384
2,plkft,2.0,0.0,1.2303513791243772
2,dkf,2.0,0.0,11.5498979593969
2,dkft,2.0,0.0,6.690193722276153
    """

st_3 = """
3,svd-static,0.0,0.0,23.18120441916196
3,svd-dynamic,0.0,0.0,22.223212955713723
3,plkf,0.0,0.0,8.68985508879634
3,plkft,0.0,0.0,0.9864716261202744
3,dkf,0.0,0.0,8.139488815961158
3,dkft,0.0,0.0,1.003510996843215
3,plkf,0.5,0.0,10.773389812524561
3,plkft,0.5,0.0,1.7983754864861252
3,dkf,0.5,0.0,5.988037498083607
3,dkft,0.5,0.0,1.9646342087041346
3,plkf,1.0,0.0,11.391012057474413
3,plkft,1.0,0.0,2.9725312435223588
3,dkf,1.0,0.0,10.688492393789105
3,dkft,1.0,0.0,1.5340237569378916
3,plkf,2.0,0.0,11.658403663370054
3,plkft,2.0,0.0,1.7740231893606386
3,dkf,2.0,0.0,9.693425987341353
3,dkft,2.0,0.0,2.459994715865283
3,plkf,0.0,0.0,8.68985508879634
3,plkft,0.0,0.0,0.9864716261202744
3,dkf,0.0,0.0,8.139488815961158
3,dkft,0.0,0.0,1.003510996843215
3,plkf,0.0,3.0,16.047266467822972
3,plkft,0.0,3.0,9.61296732502003
3,dkf,0.0,3.0,19.52574888177896
3,dkft,0.0,3.0,16.49006928083707
3,plkf,0.0,6.0,24.86634958301809
3,plkft,0.0,6.0,15.056387426031899
3,dkf,0.0,6.0,19.233415068977056
3,dkft,0.0,6.0,15.845366889704863
3,plkf,0.0,9.0,25.940370714650378
3,plkft,0.0,9.0,15.672219481267586
3,dkf,0.0,9.0,20.627099467523692
3,dkft,0.0,9.0,23.718044051140687
    """
st_4 = """
4,svd-static,0.0,0.0,9.528679527781854
4,svd-dynamic,0.0,0.0,11.557182592334211
4,plkf,0.0,0.0,1.2084868393730017
4,plkft,0.0,0.0,1.121125535396549
4,dkf,0.0,0.0,0.6753645222996345
4,dkft,0.0,0.0,0.8238434946629871
4,plkf,0.0,3.0,1.084100386849046
4,plkft,0.0,3.0,2.707917155787447
4,dkf,0.0,3.0,6.47768935995053
4,dkft,0.0,3.0,5.429136844945245
4,plkf,0.0,6.0,0.9974941624395404
4,plkft,0.0,6.0,3.7448841358068607
4,dkf,0.0,6.0,10.678514259554916
4,dkft,0.0,6.0,9.473835655563436
4,plkf,0.0,9.0,1.5276459193463974
4,plkft,0.0,9.0,5.2936498210177
4,dkf,0.0,9.0,12.24745071247022
4,dkft,0.0,9.0,12.025374808061297
4,plkf,0.0,0.0,1.2084868393730017
4,plkft,0.0,0.0,1.121125535396549
4,dkf,0.0,0.0,0.6753645222996345
4,dkft,0.0,0.0,0.8238434946629871
4,plkf,0.5,0.0,1.3289101019030658
4,plkft,0.5,0.0,2.1888984567770704
4,dkf,0.5,0.0,1.1892360732527474
4,dkft,0.5,0.0,2.1020909286487153
4,plkf,1.0,0.0,1.8213922040187442
4,plkft,1.0,0.0,2.346137638586714
4,dkf,1.0,0.0,1.3126084629392782
4,dkft,1.0,0.0,2.1726796850350527
4,plkf,2.0,0.0,1.877177395899348
4,plkft,2.0,0.0,2.119799456690606
4,dkf,2.0,0.0,1.3990139844242904
4,dkft,2.0,0.0,2.486750069171801
    """

if __name__ == "__main__":
    st = st_2
    methods = ["plkf", "plkft", "dkf", "dkft", "svd-static", "svd-dynamic"]
    # methods = ["plkf", "plkft", "dkf", "dkft"]
    markers = ["*", ".", "o", "+", "^", "x"]
    data = []
    for line in st.strip().split("\n"):
        a = 0
        SCENARIO, method, eagle_precision, detection_precision, rmse = line.split(",")
        SCENARIO = int(SCENARIO)
        method = methods.index(method)
        eagle_precision = float(eagle_precision)
        detection_precision = float(detection_precision)
        rmse = float(rmse)
        new_data = [SCENARIO, method, eagle_precision, detection_precision, rmse]
        if not new_data in data:
            data.append(new_data)
        # print(f"{SCENARIO},"
        #       f"{method},"
        #       f"{eagle_precision},"
        #       f"{detection_precision},"
        #       f"{rmse}")

    # data preproc
    data = np.array(data)
    data_nonzero_track = data[data[:, 2] == 0.0]
    data_nonzero_loc = data[data[:, 3] == 0.0]

    # limits for axis
    max_rmse = np.max(data[:, -1])
    max_rmse += max_rmse * 0.5

    # figures prepare
    fig_euclidian = plt.figure(layout='constrained', figsize=(14, 5))

    subfigs = fig_euclidian.subfigures(1, 2)
    var_localisation = subfigs[0]
    var_localisation = var_localisation.subplots(1, 1)
    var_localisation.set_ylim(0, max_rmse)
    a = list(set(data[:, 2]))
    var_localisation.set_xticks(a)

    var_tracking = subfigs[1]
    var_tracking = var_tracking.subplots(1, 1)
    var_tracking.set_ylim(0, max_rmse)
    a = list(set(data[:, 3]))
    var_tracking.set_xticks(a)

    counter = 0
    for counter in range(len(methods)):
        x1 = data_nonzero_loc[data_nonzero_loc[:, 1] == counter, 2]
        y1 = data_nonzero_loc[data_nonzero_loc[:, 1] == counter, 4]
        x2 = data_nonzero_track[data_nonzero_track[:, 1] == counter, 3]
        y2 = data_nonzero_track[data_nonzero_track[:, 1] == counter, 4]
        var_localisation.plot(x1, y1, marker=markers[counter], label=f"{methods[counter]}")
        var_tracking.plot(x2, y2, marker=markers[counter], label=f"{methods[counter]}")
        counter += 1

    var_localisation.grid(which='major', axis='both', linestyle='-')
    var_localisation.legend(fontsize="13", loc="upper right")
    var_localisation.set_xlabel('self-localisation deviation, [m]', fontsize="14")
    var_localisation.set_ylabel('RMSE, [m]', fontsize="14")

    var_tracking.grid(which='major', axis='both', linestyle='-')
    # var_tracking.legend(fontsize="10", loc="upper left")
    var_tracking.set_xlabel('visual tracking deviation, [px]', fontsize="14")
    fig_euclidian.suptitle(f"Scenario {SCENARIO}.")
    plt.show()
    fig_euclidian.savefig(f"sc_{SCENARIO}_deviation_comp.pdf", format="pdf", bbox_inches="tight")
