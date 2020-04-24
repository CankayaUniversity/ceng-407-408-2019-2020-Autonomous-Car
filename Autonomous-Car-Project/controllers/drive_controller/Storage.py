"""
    This script responsible of storing data.
    Büşra Nur Bahadır 201511006

                                            """

import os
import pickle


def storeData(key, value, file):
    # initializing data to be stored in db
    db = {}
    # if data exist append new data end write over
    if os.path.isfile(file) and os.stat(file).st_size > 0:
        with open(file, "rb") as f:
            unpickler = pickle.Unpickler(f)
            # to the value un-pickled
            db = unpickler.load()

    db[key] = value

    # Its important to use binary mode
    with open(file, 'wb') as f:
        # source, destination
        pickle.dump(db, f)


def loadData(key, file):
    # for reading also binary mode is important
    db = {}
    if os.path.getsize(file) > 0:
        with open(file, "rb") as f:
            unpickler = pickle.Unpickler(f)
            # to the value un-pickled
            db = unpickler.load()
    if key in db:
        value = db[key]
        return value
    else:
        print("value does not exists")


if __name__ == '__main__':
    storeData()
    loadData()
