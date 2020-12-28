### models.py ###

from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Column, Integer, Date

Base = declarative_base()

class MitEntry(Base):
    __tablename__ = 'mit'
    EntryId = Column(Integer, primary_key=True)
    Timestamp = Column(Date)
    
    def __repr__(self):
        return "<Book(EntryID='{}', Timestamp='{}')>"\
                .format(self.EntryId, self.Timestamp)