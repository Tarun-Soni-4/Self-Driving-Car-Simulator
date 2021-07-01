import os
import pickle

def storeData(key, value, file):
    db = loadAll(file)
    db[key] = value
    if os.path.isfile(file) and os.stat(file).st_size > 0 and os.path.getsize(file) > 0:
        with open(file, 'wb') as f:
            pickle.dump(db, f)
            
def loadAll(file):
    db = dict()
    if os.path.isfile(file) and os.stat(file).st_size > 0 and os.path.getsize(file) > 0:
        with open(file, "rb") as f:
            unpickler = pickle.Unpickler(f)
            db = unpickler.load()
            return db
    else:
        return db

def loadData(key, file):
    db = loadAll(file)
    if key in db:
        value = db[key]
        return value
    else:
        return None

def deleteData(key: str, file: str) -> None:
    db = dict()
    db = loadAll(file)
    if key in db:
        del db[key]

    with open(file, 'wb') as f:
        pickle.dump(db, f)

storeData()
loadData()
deleteData()
