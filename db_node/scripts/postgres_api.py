#!/usr/bin/env python3
DATABASE_URI = 'postgres+psycopg2://postgres:postgres@localhost:5432/x1'

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from contextlib import contextmanager

from models import MitEntry

@contextmanager
def session_scope():
    session = Session()
    try:
        yield session
        session.commit()
    except Exception:
        session.rollback()
        raise
    finally:
        session.close()

class PostgresAPI(FormalDatabaseInterface):
    def __init__(self):
        self.engine = create_engine(DATABASE_URI)
        Session = sessionmaker(bind=engine)

    def recreate_database():
        Base.metadata.drop_all(self.engine)
        Base.metadata.create_all(self.engine)

    def generate_mit(self):
        results = self.s.query(MitEntry)
        print(results)

    def write_to_db(self):
        temp = MitEntry(
            EntryId=1206
            Timestamp=datetime(2013,1,1)
        )
        self.s.add(temp)
        self.s.commit()

p = PostgresAPI()