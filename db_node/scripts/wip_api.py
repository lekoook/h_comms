DATABASE_URI = 'postgres+psycopg2://postgres:postgres@localhost:5432/x1'

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from contextlib import contextmanager
from datetime import datetime

from models import MitEntry, Base

engine = create_engine(DATABASE_URI)
Base.metadata.create_all(engine)

Session = sessionmaker(bind=engine)
s = Session()
temp = MitEntry(
    EntryId=1206,
    Timestamp=datetime(2013,1,1,1,1,1)
)
s.add(temp)
s.commit()

print(s.query(MitEntry).all())