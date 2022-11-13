/****************************************************************************
** Copyright (c) 2022, Carsten Schmidt. All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/

#include <cs/Core/QStringUtil.h>

#include "CampaignModel.h"

#include "HexChar.h"

////// Constants /////////////////////////////////////////////////////////////

constexpr char HEX_FILL = ' ';

////// public ////////////////////////////////////////////////////////////////

CampaignModel::CampaignModel(QObject *parent)
  : QAbstractTableModel(parent)
{
}

CampaignModel::~CampaignModel()
{
}

void CampaignModel::clear()
{
  beginResetModel();
  _campaign.clear();
  endResetModel();
}

void CampaignModel::set(const Campaign& c)
{
  beginResetModel();
  _campaign = c;
  endResetModel();
}

Campaign CampaignModel::campaign() const
{
  return _campaign;
}

QString CampaignModel::filename() const
{
  return cs::toQString(_campaign.filename);
}

QString CampaignModel::key() const
{
  return !_campaign.key.empty()
      ? QString::fromStdString(toHexString(_campaign.key, HEX_FILL))
      : QString();
}

std::filesystem::path CampaignModel::traceFilename(const std::size_t i) const
{
  if( i < 0  ||  i >= _campaign.entries.size() ) {
    return std::filesystem::path{};
  }

  const CampaignEntry& entry = _campaign.entries[i];
  if( !entry.exists(_campaign.filename) ) {
    return std::filesystem::path{};
  }

  return entry.filename(_campaign.filename);
}

int CampaignModel::columnCount(const QModelIndex& /*parent*/) const
{
  return Num_Columns;
}

QVariant CampaignModel::data(const QModelIndex& index, int role) const
{
  if( !index.isValid() ) {
    return QVariant();
  }
  if( role == Qt::DisplayRole ) {
    const int i = index.row();
    const int j = index.column();

    if(        j == Col_Name ) {
      return QString::fromStdString(_campaign.entries[i].name);
    } else if( j == Col_Plain ) {
      return QString::fromStdString(toHexString(_campaign.entries[i].plain, HEX_FILL));
    } else if( j == Col_Cipher ) {
      return QString::fromStdString(toHexString(_campaign.entries[i].cipher, HEX_FILL));
    }
  }
  return QVariant();
}

Qt::ItemFlags CampaignModel::flags(const QModelIndex& index) const
{
  return QAbstractTableModel::flags(index);
}

QVariant CampaignModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  if( role == Qt::DisplayRole ) {
    if(        orientation == Qt::Horizontal ) {
      if(        section == Col_Name ) {
        return tr("Name");
      } else if( section == Col_Plain ) {
        return tr("Plain");
      } else if( section == Col_Cipher ) {
        return tr("Cipher");
      }
    } else if( orientation == Qt::Vertical ) {
      return QString(QStringLiteral("%1")).arg(section + 1);
    }
  }
  return QVariant();
}

int CampaignModel::rowCount(const QModelIndex& /*parent*/) const
{
  return int(_campaign.entries.size());
}
