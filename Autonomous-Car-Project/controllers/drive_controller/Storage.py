"""
    This script responsible of storing data.
    Büşra Nur Bahadır 201511006

                                            """

import os
import pickle


# function to save data to pickle file
def storeData(key, value, file):
    # if data exist append new data end write over
    db = loadAll(file)
    db[key] = value
    if os.path.isfile(file) and os.stat(file).st_size > 0 and os.path.getsize(file) > 0:
        with open(file, 'wb') as f:
            pickle.dump(db, f)


"""------------------------------------------------------------------------------------------------------------"""


def loadAll(file):
    # initializing data to be stored in db
    db = dict()
    if os.path.isfile(file) and os.stat(file).st_size > 0 and os.path.getsize(file) > 0:
        with open(file, "rb") as f:
            unpickler = pickle.Unpickler(f)
            db = unpickler.load()
            return db
    else:
        return db


"""------------------------------------------------------------------------------------------------------------"""


# function to load data from pickle file
def loadData(key, file):
    db = loadAll(file)
    if key in db:
        value = db[key]
        return value
    else:
        # print("value does not exists")
        return None


"""------------------------------------------------------------------------------------------------------------"""


def deleteData(key: str, file: str) -> None:
    db = dict()
    db = loadAll(file)
    if key in db:
        del db[key]

    with open(file, 'wb') as f:
        pickle.dump(db, f)


"""------------------------------------------------------------------------------------------------------------"""

if __name__ == '__main__':
    storeData()
    loadData()
    deleteData()
